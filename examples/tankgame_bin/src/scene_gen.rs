use bevy::render::texture::Image;
use bevy_vello::{
    prelude::*,
    vello::{kurbo::Affine, peniko::PBRImages},
};
use std::str;

fn _export_default_image_scene(albedo: &[u8], is_masked: bool) {
    let mut base_image = vello::decode_image(albedo).unwrap();
    let width = base_image.width as f64;
    let height = base_image.height as f64;

    base_image = if is_masked {
        base_image.with_usage(peniko::ImageUsageType::MASKED)
    } else {
        base_image.with_usage(peniko::ImageUsageType::TRANSPARENT)
    };

    let mut scene = VelloScene::default();
    scene.write_trace = true;
    scene.draw_image(
        &base_image,
        Affine::translate((-0.5 * width, -0.5 * height)),
    );
}

fn _export_default_image_scene_with_name(name: &str, is_masked: bool) {
    use std::env;
    let exe_dir = env::current_dir()
        .expect("Failed to get executable path")
        .parent()
        .expect("Failed to get parent directory")
        .to_path_buf();
    let temp = exe_dir.join("tankgame_bin\\assets\\image");

    let base_name = format!("{}{}", name, ".png");

    let base = temp.join(base_name);
    println!("{:?},\n", base);
    use std::fs;
    let tank = fs::read(base).expect("Failed to read file");
    _export_default_image_scene(&tank, is_masked);
}

fn _export_default_pbr_scene(albedo: &[u8], normal: &[u8], metallic: f32, roughness: f32) {
    let base_image = vello::decode_image(albedo).unwrap();
    let base_normal_image = vello::decode_image(normal).unwrap();
    let width = base_image.width as f64;
    let height = base_image.height as f64;
    let pbr = PBRImages::new(base_image, base_normal_image, metallic, roughness);
    let mut scene = VelloScene::default();
    scene.write_trace = true;
    scene.draw_image_with_normal(&pbr, Affine::translate((-0.5 * width, -0.5 * height)));
}

fn _export_default_pbr_scene_with_name(name: &str, metallic: f32, roughness: f32) {
    use std::env;
    let exe_dir = env::current_dir()
        .expect("Failed to get executable path")
        .parent()
        .expect("Failed to get parent directory")
        .to_path_buf();
    let temp = exe_dir.join("tankgame_bin\\assets\\pbr");

    let base_name = format!("{}{}", name, ".png");
    let normal_name = format!("{}{}", name, "_normal.png");

    let base = temp.join(base_name);
    let normal = temp.join(normal_name);
    println!("{:?},\n {:?}", base, normal);
    use std::fs;
    let tank = fs::read(base).expect("Failed to read file");
    let tank_normals = fs::read(normal).expect("Failed to read file");
    _export_default_pbr_scene(&tank, &tank_normals, metallic, roughness);
}

fn _extract_last_three_numbers(s: &str) -> Option<(u32, u32, u32)> {
    let numbers: Vec<u32> = s
        .split('_')
        .filter_map(|part| part.parse::<u32>().ok()) // Keep only successfully parsed numbers
        .collect();

    if numbers.len() >= 3 {
        Some((
            numbers[numbers.len() - 3],
            numbers[numbers.len() - 2],
            numbers[numbers.len() - 1],
        ))
    } else {
        None // Not enough numbers found
    }
}

fn _export_default_sprite_sheet_scene(
    albedo: &[u8],
    sprite_sheet_play_config: peniko::SpriteSheetPlayConfig,
    unlit: bool,
    columns: u32,
    rows: u32,
    frame_count: u32,
) {
    let base_image = vello::decode_image(albedo).unwrap();
    let base_image = if unlit {
        base_image.with_usage(peniko::ImageUsageType::TRANSPARENT)
    } else {
        base_image.with_usage(peniko::ImageUsageType::MASKED)
    };
    let width = base_image.width / columns;
    let height = base_image.height / rows;
    let frame_count = frame_count;
    let base_image = base_image.with_sprite_sheet(Some(peniko::SpriteSheet {
        width,
        height,
        frame_count,
    }));
    let mut scene = VelloScene::default();
    scene.write_trace = true;
    scene.draw_sprite_sheet(
        &base_image,
        Affine::translate((-0.5 * (width as f64), -0.5 * (height as f64))),
        sprite_sheet_play_config,
    );
}

fn _export_default_sprite_sheet_scene_with_name(
    name: &str,
    spirte_sheet_play_config: peniko::SpriteSheetPlayConfig,
    unlit: bool,
) {
    use std::env;
    let exe_dir = env::current_dir()
        .expect("Failed to get executable path")
        .parent()
        .expect("Failed to get parent directory")
        .to_path_buf();
    let temp = exe_dir.join("tankgame_bin\\assets\\sprites");

    let base_name = format!("{}{}", name, ".png");

    let base = temp.join(base_name);
    println!("{:?},\n", base);
    let (colums, rows, count) = _extract_last_three_numbers(name).expect("wrong file name");
    use std::fs;
    let tank = fs::read(base).expect("Failed to read file");
    _export_default_sprite_sheet_scene(&tank, spirte_sheet_play_config, unlit, colums, rows, count);
}

use std::fs;

fn _change_file_name(name: &str) -> std::io::Result<()> {
    use std::env;
    let exe_dir = env::current_dir()
        .expect("Failed to get executable path")
        .parent()
        .expect("Failed to get parent directory")
        .to_path_buf();
    let temp = exe_dir.join("tankgame_bin");
    let old_name = temp.join("trace.txt");
    let target_name = format!("{}{}", name, ".scene");
    let new_name = temp.join(target_name);

    // Rename the file
    fs::rename(old_name, new_name)?;
    Ok(())
}

const PBRS: [&str; 5] = ["base", "turrent", "gun", "stone", "tree"];
#[test]
fn export_pbr_scene() {
    let i = PBRS[0];
    _export_default_pbr_scene_with_name(i, 0.1, 0.5);
    _change_file_name(i).unwrap();
}

const IMAGES: [&str; 1] = ["blood"];
#[test]
fn export_image_scene() {
    let i = IMAGES[0];
    _export_default_image_scene_with_name(i, false);
    _change_file_name(i).unwrap();
}

const SPRITES: [&str; 5] = [
    "zombie_attack_3_3_9",
    "zombie_idle_5_4_17",
    "zombie_move_5_4_17",
    "gunflare_2_2_4",
    "gunfire_2_2_4",
];
#[test]
fn export_sprit_sheet_effect() {
    let i = SPRITES[4];
    // we want the character anim to inteact with the light and shadow.
    _export_default_sprite_sheet_scene_with_name(
        i,
        peniko::SpriteSheetPlayConfig {
            fps: 6.0,
            start_time: 0.0,
            play_in_loop: true,
        },
        true,
    );
    _change_file_name(i).unwrap();
}
