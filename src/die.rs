use rand::{rngs::StdRng, Rng, SeedableRng};
use rapier3d::prelude::*;
use std::env;
#[cfg(feature = "randomorg")]
use std::time::Duration;

const DT: f32 = 1.0 / 120.0;

struct Config {
    sides: usize,
    duration: f32,
    seed: Option<u64>,
    intention: Option<String>,
    random_org: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = parse_args()?;
    let seed = config.seed.unwrap_or_else(|| entropy_seed(config.random_org));
    let result = roll(&config, mix_intention(seed, config.intention.as_deref()));
    if let Some(intention) = &config.intention {
        println!("Intention: {intention}");
    }
    println!("d{} result: {}", config.sides, result);
    Ok(())
}

fn parse_args() -> Result<Config, Box<dyn std::error::Error>> {
    let mut config = Config { sides: 6, duration: 8.0, seed: None, intention: None, random_org: false };
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || -> Result<String, Box<dyn std::error::Error>> {
            args.next().ok_or_else(|| format!("missing value for {arg}").into())
        };
        match arg.as_str() {
            "--sides" | "-s" => config.sides = value()?.parse()?,
            "--duration" | "-d" => config.duration = value()?.parse()?,
            "--seed" => config.seed = Some(value()?.parse()?),
            "--intention" | "-i" => config.intention = Some(value()?),
            "--random-org" => config.random_org = true,
            "--help" | "-h" => {
                println!("Physically simulated polygonal die\n  --sides N       Number of faces/outcomes (5 or more; default 6)\n  --duration SEC  Simulation time (default 8)\n  --seed N        Repeatable random seed\n  --intention, -i TEXT  Question or intention for the roll\n  --random-org    Use random.org (build with --features randomorg)");
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if config.sides < 5 {
        return Err("--sides must be at least 5 (the die is a polygonal prism)".into());
    }
    if !config.duration.is_finite() || config.duration <= 0.0 {
        return Err("--duration must be a finite positive number".into());
    }
    Ok(config)
}

fn entropy_seed(random_org: bool) -> u64 {
    if random_org {
        #[cfg(feature = "randomorg")]
        {
            // The unauthenticated integer endpoint caps each value at 1e9.
            // Two values provide enough entropy for a 64-bit seed.
            if let Ok(response) = ureq::get("https://www.random.org/integers/?num=2&min=0&max=1000000000&col=1&base=10&format=plain&rnd=new")
                .timeout(Duration::from_secs(5)).call() {
                if let Ok(value) = response.into_string() {
                    let mut numbers = value.split_whitespace().filter_map(|part| part.parse::<u32>().ok());
                    if let (Some(high), Some(low)) = (numbers.next(), numbers.next()) {
                        return (u64::from(high) << 32) | u64::from(low);
                    }
                }
            }
            eprintln!("random.org unavailable; using operating-system randomness");
        }
        #[cfg(not(feature = "randomorg"))]
        eprintln!("random.org support is disabled; build with --features randomorg. Using operating-system randomness.");
    }
    let mut bytes = [0_u8; 8];
    getrandom::getrandom(&mut bytes).expect("operating-system random source unavailable");
    u64::from_ne_bytes(bytes)
}

// Fold every intention byte into the seed while retaining the random seed as
// the source of per-roll variation. This is a small FNV-1a style mixer.
fn mix_intention(seed: u64, intention: Option<&str>) -> u64 {
    let Some(intention) = intention else { return seed };
    let mut mixed = seed ^ 0xcbf29ce484222325;
    for byte in intention.bytes() {
        mixed ^= u64::from(byte);
        mixed = mixed.wrapping_mul(0x100000001b3);
    }
    mixed
}

fn roll(config: &Config, seed: u64) -> usize {
    let mut rng = StdRng::seed_from_u64(seed);
    let n = config.sides - 2; // An n-gonal prism has n side faces and two ends.
    let radius = 0.02_f32; // 2 cm circumradius, appropriate for a tabletop die.
    let flat_dist = radius * (std::f32::consts::PI / n as f32).cos();
    // A d6 is an exact cube. Other counts use a practical aspect ratio between
    // end faces and side faces; this does not guarantee statistically fair rolls.
    let aspect = if n == 4 {
        1.0
    } else {
        (4.0 / n as f32).sqrt().clamp(0.6, 1.4)
    };
    let half_height = flat_dist * aspect;
    let mut vertices = Vec::with_capacity(2 * n);
    for y in [-half_height, half_height] {
        for i in 0..n {
            let angle = std::f32::consts::TAU * i as f32 / n as f32;
            vertices.push(point![radius * angle.cos(), y, radius * angle.sin()]);
        }
    }
    let hull = ColliderBuilder::convex_hull(&vertices).expect("prism vertices form a convex hull");

    let mut pipeline = PhysicsPipeline::new();
    let gravity = vector![0.0, -9.81, 0.0];
    let integration = IntegrationParameters { dt: DT, ..IntegrationParameters::default() };
    let mut islands = IslandManager::new();
    let mut broad = BroadPhaseMultiSap::new();
    let mut narrow = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = ImpulseJointSet::new();
    let mut multibody = MultibodyJointSet::new();
    let mut ccd = CCDSolver::new();

    // One-meter slab centered at -0.5 m keeps the tabletop at y=0 and deep
    // enough to contain the die if a fast step ever passes the top surface.
    let ground = bodies.insert(RigidBodyBuilder::fixed().translation(vector![0.0, -0.5, 0.0]).build());
    colliders.insert_with_parent(ColliderBuilder::cuboid(1.0, 0.5, 1.0).friction(0.75).restitution(0.18).build(), ground, &mut bodies);
    let body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 0.25, 0.0])
        .rotation(vector![rng.gen_range(-2.8..2.8), rng.gen_range(-2.8..2.8), rng.gen_range(-2.8..2.8)])
        .linvel(vector![rng.gen_range(-1.8..1.8), rng.gen_range(-0.5..1.5), rng.gen_range(-1.8..1.8)])
        .angvel(vector![rng.gen_range(-9.0..9.0), rng.gen_range(-9.0..9.0), rng.gen_range(-9.0..9.0)])
        .linear_damping(0.18)
        .angular_damping(0.35)
        .can_sleep(false)
        .build();
    let handle = bodies.insert(body);
    colliders.insert_with_parent(hull.friction(0.65).restitution(0.12).density(1150.0).build(), handle, &mut bodies);

    let steps = (config.duration / DT).ceil() as usize;
    for _ in 0..steps {
        pipeline.step(&gravity, &integration, &mut islands, &mut broad, &mut narrow,
            &mut bodies, &mut colliders, &mut joints, &mut multibody, &mut ccd,
            None, &(), &());
    }

    // Choose the face whose outward normal points most upward in the settled pose.
    let rotation = bodies[handle].rotation();
    let mut normals = Vec::with_capacity(config.sides);
    normals.push(vector![0.0, 1.0, 0.0]); // outcome 1: upper end
    normals.push(vector![0.0, -1.0, 0.0]); // outcome 2: lower end
    for i in 0..n {
        let angle = std::f32::consts::TAU * (i as f32 + 0.5) / n as f32;
        normals.push(vector![angle.cos(), 0.0, angle.sin()]);
    }
    normals.iter().enumerate()
        .max_by(|(_, a), (_, b)| {
            let ay = (rotation * **a).y;
            let by = (rotation * **b).y;
            ay.partial_cmp(&by).unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(index, _)| index + 1)
        .unwrap_or(1)
}
