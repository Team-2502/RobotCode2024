use std::{sync::Arc, collections::HashMap, time::Instant, };
use axum::{Router, extract::{State, Path}, routing::{get, post}, response::{Html, IntoResponse}, Json, http::{StatusCode, Response, HeaderValue, header}, body::Body,  };
use include_dir::Dir;
use lazy_static::lazy_static;
use num_traits::{ToPrimitive, FromPrimitive};
use serde::{Serialize, Deserialize};
use tokio::sync::RwLock;

use crate::auto::Auto;

pub type TelemetryStore = Arc<RwLock<Telemetry>>;

lazy_static!{
        pub static ref TELEMETRY: TelemetryStore = {

        let mut telemetry: Telemetry = Default::default();
        telemetry.data.insert("auto chooser".to_owned(), Data::Picker(Auto::picker()));
        Arc::new(RwLock::new(telemetry))
    };
}

#[derive(Deserialize, Serialize, Clone)]
pub enum Data {
    Number(f64),
    Bool(bool),
    Text(String),
    Pose(Pose),
    Picker(Picker),
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

#[derive(Serialize, Deserialize, Default, Clone)]
pub struct Picker {
    pub options: Vec<String>,
    pub selected: String, // this is a number
}

pub fn server() -> Router<TelemetryStore> {
    let router = Router::new()
        .route("/:path/:path/:path/:path/:path", get(frontend)) // I'm wanting to kill myself ;)
        .route("/:path/:path/:path/:path", get(frontend))
        .route("/:path/:path/:path", get(frontend))
        .route("/:path/:path", get(frontend))
        .route("/:path", get(frontend)) // I want to kill myself :)
        .route("/", get(frontend)) // I want to kill myself :)
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

static STATIC_DIR: Dir<'_> = include_dir::include_dir!("$CARGO_MANIFEST_DIR/talon-board/out");

// thanks https://bloerg.net/posts/serve-static-content-with-axum/
async fn frontend(Path(path): Path<Vec<String>>) -> impl IntoResponse {
    let path = path.join("/");
    let mut path = path.trim_start_matches('/');
    if path.is_empty() {
        path = "index.html";
    }
    let mime_type = mime_guess::from_path(path).first_or_text_plain();

    match STATIC_DIR.get_file(path) {
        None => {  Response::builder()
            .status(StatusCode::NOT_FOUND)
            .body(Body::empty())
            .unwrap() },
        Some(file) => Response::builder()
            .status(StatusCode::OK)
            .header(
                header::CONTENT_TYPE,
                HeaderValue::from_str(mime_type.as_ref()).unwrap(),
            )
            .body(Body::from(file.contents()))
            .unwrap(),
    }
}

pub async fn put_number(key: &str, value: f64) {
    TELEMETRY.write().await.data.insert(key.to_owned(), Data::Number(value));
}

pub async fn put_bool(key: &str, value: bool) {
    TELEMETRY.write().await.data.insert(key.to_owned(), Data::Bool(value));
}
