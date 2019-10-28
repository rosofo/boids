use serde::{Deserialize, Serialize};
use std::fs;
use hotwatch::*;
use std::sync::mpsc;

#[derive(Serialize, Deserialize)]
pub struct Config {
    pub influence_radius: f32,
    pub drag_coefficient: f32,
    pub population: u32,
    pub max_force: f32,
    pub max_ang_vel: f32,
}

pub fn read_config(path: &str) -> Config {
    toml::from_slice(&fs::read(path).expect("Config file doesn't exist."))
        .expect("Config file unreadable.")
}

pub fn watch_config(path: &'static str) -> (mpsc::Receiver<Config>, Hotwatch) {
    let (sender, receiver) = mpsc::channel();
    let mut hotwatch = Hotwatch::new().unwrap();
    hotwatch.watch(path, move |event| {
        if let Event::Write(_) = event {
            let config = read_config(path);
            sender.send(config).unwrap();
        }
    }).unwrap();
    (receiver, hotwatch)
}