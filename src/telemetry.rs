use std::{sync::Arc, collections::HashMap, time::Instant};
use axum::{Router, extract::{State, Path}, routing::{get, post}, response::Html, Json};
use lazy_static::lazy_static;
use num_traits::{ToPrimitive, FromPrimitive};
use serde::{Serialize, Deserialize};
use tokio::sync::RwLock;

use crate::auto::Auto;

pub type TelemetryStore = Arc<RwLock<Telemetry>>;

lazy_static!{
        pub static ref TELEMETRY: TelemetryStore = {
        Arc::new(RwLock::new(Default::default()))
    };
}

#[derive(Deserialize, Serialize, Clone)]
pub enum Data {
    Number(f64),
    Bool(bool),
    Text(String),
    Pose(Pose),
}

#[derive(Default)]
pub struct Telemetry {
    pub auto: Auto,
    pub data: HashMap<String, Data>,
    pub apriltag_pose: Option<(Instant, Pose)>,
}

#[derive(Serialize, Deserialize, Default, Clone)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64, // degrees
}

pub fn server() -> Router<TelemetryStore> {
    let router = Router::new()
        .route("/", get(frontend))
        .route("/get_auto", get(get_auto))
        .route("/get_auto_name", get(get_auto_name))
        .route("/get_auto/:id", get(get_auto_name_by_id))
        .route("/set_auto/:id", get(set_auto)) // words have no meaning :)
        .route("/set_position", post(set_position))
        .route("/get/:key", get(get_key))
        .route("/set/:key", post(set_key))
        .route("/get_keys", get(get_keys));

    router
}

async fn set_key(
    Path(key): Path<String>,
    State(state): State<TelemetryStore>,
    Json(data): Json<Data>,
) -> &'static str {
    state.write().await.data.insert(key, data);
    "Success"
}

async fn get_key(State(state): State<TelemetryStore>,
    Path(key): Path<String>,
) -> Json<Option<Data>> {
    Json(state.read().await.data.get(&key).cloned())
}

async fn get_keys(State(state): State<TelemetryStore>) -> Json<Vec<String>> {
    Json(state.read().await.data.keys().cloned().collect())
}

async fn set_position(
    State(state): State<TelemetryStore>,
    Json(pose): Json<Pose>,
    ) -> &'static str {
    let pose = (Instant::now(), pose);
    println!("apriltag pose at x{} y{}", pose.1.x, pose.1.y);
    state.write().await.apriltag_pose = Some(pose);

    "written"
}

async fn frontend() -> Html<&'static str> {
    Html(include_str!("../client/selector.html"))
}

async fn get_auto_name(
    State(state): State<TelemetryStore>) -> &'static str {
    state.read().await.auto.to_owned().name()
}

async fn get_auto(
    State(state): State<TelemetryStore>) -> String {
    state.read().await.auto.to_owned().to_usize().unwrap().to_string()
}

async fn get_auto_name_by_id(
    Path(auto): Path<usize>,
    ) -> &'static str {
    if let Some(auto) = Auto::from_usize(auto) {
        auto.name()
    } else {
        "Not real"
    }
}

async fn set_auto(
    State(state): State<TelemetryStore>,
    Path(auto): Path<usize>,
    ) -> &'static str {
    if let Some(auto) = Auto::from_usize(auto) {
        state.write().await.auto = auto;

        "Success"
    } else {
        "Failure"
    }

}
