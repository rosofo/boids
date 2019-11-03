use hotwatch::*;
use serde::{Deserialize, Serialize};
use std::fs;
use std::sync::mpsc;

#[derive(Serialize, Deserialize)]
pub struct Config {
    pub influence_radius: f32,
    pub drag_coefficient: f32,
    pub population: u32,
    pub max_force: f32,
    pub max_ang_vel: f32,
    pub behaviour_active: bool,
}

pub fn read(path: &str) -> Config {
    toml::from_slice(&fs::read(path).expect("Config file doesn't exist."))
        .expect("Config file unreadable.")
}

pub fn watch(path: String) -> (mpsc::Receiver<Config>, Hotwatch) {
    let (sender, receiver) = mpsc::channel();
    let mut hotwatch = Hotwatch::new().unwrap();

    hotwatch
        .watch(path.clone(), move |event| {
            if let Event::Write(_) = event {
                let config = read(&path);
                sender.send(config).unwrap();
            }
        })
        .unwrap();

    (receiver, hotwatch)
}
