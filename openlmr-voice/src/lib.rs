#![no_std]
use core::{concat, env, include, include_bytes};

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
