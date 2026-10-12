use rapier3d::prelude::*;
use std::env;
#[cfg(feature = "randomorg")]
use std::time::Duration;

const DT: f32 = 1.0 / 120.0;

#[derive(Debug)]
struct Config {
    length: f32,
    mass: f32,
    drag: f32,
    wind: f32,
    duration: f32,
    question: String,
    random_org: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self { length: 1.8, mass: 0.085, drag: 0.18, wind: 0.0,
            duration: 6.2, question: String::new(), random_org: false }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = parse_args()?;
    let seed = entropy_seed(config.random_org);
    let reading = simulate(&config, seed);
    println!("Question: {}", if config.question.is_empty() { "(none)" } else { &config.question });
    println!("Average swing position: x={:.3} m, z={:.3} m", reading.avg_x, reading.avg_z);
    println!("Answer: {}", reading.answer);
    println!("{}", match reading.answer {
        "YES" => "The movement settles with an open current.",
        "NO" => "The motion closes toward a boundary.",
        _ => "The field remains unsettled — return when clear.",
    });
    Ok(())
}

fn parse_args() -> Result<Config, Box<dyn std::error::Error>> {
    let mut c = Config::default();
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || -> Result<String, Box<dyn std::error::Error>> {
            args.next().ok_or_else(|| format!("missing value for {arg}").into())
        };
        match arg.as_str() {
            "--length" => c.length = value()?.parse()?,
            "--mass" => c.mass = value()?.parse::<f32>()? / 1000.0,
            "--drag" => c.drag = value()?.parse()?,
            "--wind" => c.wind = value()?.parse()?,
            "--duration" => c.duration = value()?.parse()?,
            "--question" => c.question = value()?,
            "--random-org" => c.random_org = true,
            "--help" | "-h" => {
                println!("3D pendulum simulation\n  --length METERS --mass GRAMS --drag VALUE --wind MPS\n  --duration SECONDS --question TEXT --random-org (build with --features randomorg)");
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if !(c.length.is_finite() && c.length > 0.1 && c.mass.is_finite() && c.mass > 0.0
        && c.drag.is_finite() && c.drag >= 0.0 && c.wind.is_finite()
        && c.duration.is_finite() && c.duration > 0.0) {
        return Err("length, mass, drag, wind, and duration must be finite; length, mass, duration must be positive and drag nonnegative".into());
    }
    Ok(c)
}

fn entropy_seed(random_org: bool) -> u32 {
    if random_org {
        #[cfg(feature = "randomorg")]
        {
            if let Ok(response) = ureq::get("https://www.random.org/integers/?num=1&min=1&max=999999&col=1&base=10&format=plain&rnd=new")
                .timeout(Duration::from_secs(5)).call() {
                if let Ok(value) = response.into_string() {
                    if let Ok(seed) = value.trim().parse::<u32>() { return seed; }
                }
            }
            eprintln!("random.org unavailable; using operating-system randomness");
        }
        #[cfg(not(feature = "randomorg"))]
        eprintln!("random.org support is disabled; build with --features randomorg. Using operating-system randomness.");
    }
    let mut bytes = [0_u8; 4];
    getrandom::getrandom(&mut bytes).expect("operating-system random source unavailable");
    u32::from_ne_bytes(bytes)
}

struct Reading { avg_x: f32, avg_z: f32, answer: &'static str }

fn simulate(c: &Config, seed: u32) -> Reading {
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
    let pivot = vector![0.0, 1.8, 0.0];
    let x = (0.52_f32).min(c.length * 0.28);
    let y = 1.8 - (c.length * c.length - x * x).max(0.01).sqrt();
    let body = RigidBodyBuilder::dynamic().translation(vector![x, y, 0.0])
        .linear_damping(0.0).angular_damping(0.55).additional_mass(c.mass)
        .can_sleep(false).build();
    let handle = bodies.insert(body);
    colliders.insert_with_parent(ColliderBuilder::ball(0.015).density(1.0).build(), handle, &mut bodies);
    bodies[handle].set_linvel(vector![0.42, 0.127, 0.0], true);
    let anchor = bodies.insert(RigidBodyBuilder::fixed().translation(pivot).build());
    let bob_to_pivot = pivot - *bodies[handle].translation();
    let joint = SphericalJointBuilder::new()
        .local_anchor1(point![0.0, 0.0, 0.0])
        .local_anchor2(point![bob_to_pivot.x, bob_to_pivot.y, bob_to_pivot.z]);
    joints.insert(anchor, handle, joint, true);
    let steps = (c.duration / DT).ceil() as usize;
    let mut sum_x = 0.0;
    let mut sum_z = 0.0;
    let mut sum_spin = 0.0;
    for i in 0..steps {
        let t = i as f32 * DT;
        let phase = (seed as f32 / u32::MAX as f32) * 10.0;
        let wind = vector![c.wind * (0.65 + 0.35 * (t / 1.1 + phase).sin()), 0.0, c.wind * 0.26];
        let velocity = *bodies[handle].linvel();
        let relative = velocity - wind;
        let speed = relative.norm();
        let drag_force = relative * (-c.drag * speed * 1.7);
        let flutter = 0.035 + c.wind * 0.04;
        let force = drag_force + vector![(t * 1.7 + phase * 0.8).sin() * flutter, 0.0,
            (t * 1.31 + phase * 0.5).cos() * flutter];
        bodies[handle].add_force(force, true);
        pipeline.step(&gravity, &integration, &mut islands, &mut broad, &mut narrow,
            &mut bodies, &mut colliders, &mut joints, &mut multibody, &mut ccd,
            None, &(), &());
        let p = *bodies[handle].translation();
        sum_x += p.x;
        sum_z += p.z;
        let radius = p - pivot;
        let velocity = *bodies[handle].linvel();
        sum_spin += radius.x * velocity.z - radius.z * velocity.x;
    }
    let avg_x = sum_x / steps as f32;
    let avg_z = sum_z / steps as f32;
    let p = *bodies[handle].translation() - pivot;
    let v = *bodies[handle].linvel();
    let avg_spin = sum_spin / steps as f32;
    let answer = if !p.norm().is_finite() || !v.norm().is_finite() {
        "UNCLEAR"
    } else if v.norm() < 0.02 {
        "UNCLEAR"
    } else if avg_spin > 0.015 {
        "YES"
    } else if avg_spin < -0.015 {
        "NO"
    } else if p.x.abs() > p.z.abs() {
        "NO"
    } else {
        "YES"
    };
    Reading { avg_x, avg_z, answer }
}
