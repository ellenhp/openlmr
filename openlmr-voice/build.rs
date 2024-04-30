use std::{
    env,
    fs::{DirBuilder, File},
    io::{Read, Write},
    path::Path,
    process::Command,
};

use codec2::Codec2Mode;
use toml::Table;

fn main() {
    println!("cargo::rerun-if-changed=build.rs,prompts/main.toml");

    let mut buf = String::new();
    File::open("./prompts/main.toml")
        .unwrap()
        .read_to_string(&mut buf)
        .unwrap();

    let table: Table = toml::from_str(&buf).unwrap();

    let prompts = table.get("prompts").unwrap().as_table().unwrap();
    let out_dir = env::var("OUT_DIR").unwrap();

    let mut bindings_file = File::create(format!("{}/bindings.rs", out_dir)).unwrap();

    let _ = DirBuilder::new().create(format!("{}/audio", out_dir));
    for (key, prompt) in prompts {
        let prompt = prompt.as_str().unwrap();
        Command::new("espeak")
            .arg(format!("\"{}\"", prompt))
            .arg("-w")
            .arg(format!("{}/{}.wav", out_dir, key))
            .output()
            .expect("Failed to execute espeak");

        let mut wav_file = File::open(Path::new(&format!("{}/{}.wav", out_dir, key))).unwrap();
        let (_wav_header, wav_data) = wav::read(&mut wav_file).unwrap();

        let wav_data = wav_data.as_sixteen().unwrap();
        let mut resampled_data = vec![];
        for chunk in wav_data.chunks_exact(3) {
            resampled_data.push((chunk.iter().map(|v| *v as i32).sum::<i32>() / 3) as i16);
        }

        let mut codec = codec2::Codec2::new(Codec2Mode::MODE_1600);
        for _ in resampled_data.len() % codec.samples_per_frame()..codec.samples_per_frame() {
            resampled_data.push(0);
        }
        let bits = (resampled_data.len() + codec.samples_per_frame() - 1)
            / codec.samples_per_frame()
            * codec.bits_per_frame();

        dbg!(bits);
        let mut c2_data = vec![0; (bits + 7) / 8];
        for chunk in
            0..(resampled_data.len() + codec.samples_per_frame() - 1) / codec.samples_per_frame()
        {
            dbg!(chunk, codec.bits_per_frame());
            let bytes_per_frame = 8;
            codec.encode(
                &mut c2_data[chunk * bytes_per_frame..(chunk + 1) * bytes_per_frame],
                &resampled_data
                    [chunk * codec.samples_per_frame()..(chunk + 1) * codec.samples_per_frame()],
            );
        }
        let mut c2_file = File::create(Path::new(&format!("{}/audio/{}", out_dir, key))).unwrap();
        c2_file.write_all(&c2_data).unwrap();

        bindings_file
            .write_fmt(format_args!(
                "pub static {}: &[u8; {}] = include_bytes!(\"{}\");\n",
                key.to_uppercase(),
                c2_data.len(),
                format!("{}/audio/{}", out_dir, key)
            ))
            .unwrap();
    }
}
