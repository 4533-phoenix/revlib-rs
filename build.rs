extern crate cc;
extern crate pkg_config;

use std::path::Path;

use cc::Build;
use pkg_config::Config;

fn main() {
  Build::new()
    .cpp(true)
    .file("src/bindings.cpp")
    .compile("revlib");
}
