use image::ImageFormat;
use std::f32;
use std::path::Path;

#[derive(Clone)]
pub struct Material {
    pub ambient: [f32; 3],
    pub diffuse: [f32; 3],
    pub specular: [f32; 3],
}

#[derive(Clone)]
pub enum AAMethod {
    None,
    FXAA,
}

#[derive(Clone)]
pub struct Config {
    pub model_filename: String,
    pub img_filename: String,
    pub format: ImageFormat,
    pub width: u32,
    pub height: u32,
    pub visible: bool,
    pub verbosity: usize,
    pub material: Material,
    pub background: (f32, f32, f32, f32),
    pub aamethod: AAMethod,
    pub recalc_normals: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            model_filename: "".to_string(),
            img_filename: "".to_string(),
            format: ImageFormat::Png,
            width: 1024,
            height: 768,
            visible: false,
            verbosity: 0,
            material: Material {
                ambient: [0.00, 0.13, 0.26],
                diffuse: [0.38, 0.63, 1.00],
                specular: [1.00, 1.00, 1.00],
            },
            background: (0.0, 0.0, 0.0, 0.0),
            aamethod: AAMethod::FXAA,
            recalc_normals: false,
        }
    }
}

impl Config {
    pub fn new() -> Config {
        // Define command line arguments
        let mut matches = clap::Command::new(env!("CARGO_PKG_NAME"))
            .version(env!("CARGO_PKG_VERSION"))
            .author(env!("CARGO_PKG_AUTHORS"))
            .arg(
                clap::Arg::new("MODEL_FILE")
                    .help("3D model file (STL, OBJ, 3MF or STEP). Use - to read an STL from stdin instead of a file.")
                    .required(true)
                    .index(1),
            )
            .arg(
                clap::Arg::new("IMG_FILE")
                    .help("Thumbnail image file. Use - to write to stdout instead of a file.")
                    .required(true)
                    .index(2),
            )
            .arg(
                clap::Arg::new("format")
                    .help("The format of the image file. If not specified it will be determined from the file extension, or default to PNG if there is no extension. Supported formats: PNG, JPEG, GIF, ICO, BMP")
                    .short('f')
                    .long("format")
                    .value_parser(parse_format)
            )
            .arg(
                clap::Arg::new("size")
                    .help("Width and height of the thumbnail in pixels (it is always square). Defaults to 1024x768 when not given.")
                    .short('s')
                    .long("size")
                    .value_parser(clap::value_parser!(u32).range(1..))
            )
            .arg(
                clap::Arg::new("visible")
                    .help("Display the thumbnail in a window instead of saving a file")
                    .short('x')
                    .action(clap::ArgAction::SetTrue)
            )
            .arg(
                clap::Arg::new("verbosity")
                    .short('v')
                    .action(clap::ArgAction::Count)
                    .help("Increase message verbosity")
            )
            .arg(
                clap::Arg::new("material")
                    .help("Colors for rendering the mesh using the Phong reflection model. Requires 3 colors as rgb hex values: ambient, diffuse, and specular. Defaults to blue.")
                    .short('m')
                    .long("material")
                    .num_args(3)
                    .value_names(["ambient", "diffuse", "specular"])
                    .value_parser(parse_rgb)
            )
            .arg(
                clap::Arg::new("background")
                    .help("The background color with transparency (rgba hex). Default is 00000000 (fully transparent).")
                    .short('b')
                    .long("background")
                    .value_parser(parse_rgba)
            )
            .arg(
                clap::Arg::new("aamethod")
                    .help("Anti-aliasing method. Default is FXAA, which is fast but may introduce artifacts.")
                    .short('a')
                    .long("antialiasing")
                    .value_parser(["none", "fxaa"]),
            )
            .arg(
                clap::Arg::new("recalc_normals")
                    .help("Force recalculation of face normals. Use when dealing with malformed STL files.")
                    .long("recalc-normals")
                    .action(clap::ArgAction::SetTrue)
            )
            .get_matches();

        let mut c = Config {
            ..Default::default()
        };

        c.model_filename = matches
            .remove_one::<String>("MODEL_FILE")
            .expect("MODEL_FILE not provided");
        c.img_filename = matches
            .remove_one::<String>("IMG_FILE")
            .expect("IMG_FILE not provided");
        match matches.get_one::<ImageFormat>("format") {
            Some(x) => c.format = *x,
            None => {
                if let Some(ext) = Path::new(&c.img_filename).extension() {
                    c.format = match_format(&ext.to_string_lossy());
                }
            }
        };

        if let Some(&size) = matches.get_one::<u32>("size") {
            c.width = size;
            c.height = size;
        }

        c.visible = matches.get_flag("visible");
        c.verbosity = matches.get_count("verbosity") as usize;
        if let Some(mut colors) = matches.get_many::<[f32; 3]>("material") {
            // clap guarantees exactly 3 values
            c.material = Material {
                ambient: *colors.next().unwrap(),
                diffuse: *colors.next().unwrap(),
                specular: *colors.next().unwrap(),
            };
        }
        if let Some(&x) = matches.get_one::<(f32, f32, f32, f32)>("background") {
            c.background = x;
        }
        if let Some(x) = matches.get_one::<String>("aamethod") {
            match x.as_str() {
                "none" => c.aamethod = AAMethod::None,
                "fxaa" => c.aamethod = AAMethod::FXAA,
                _ => unreachable!(),
            }
        }
        c.recalc_normals = matches.get_flag("recalc_normals");

        c
    }
}

fn format_from_str(ext: &str) -> Option<ImageFormat> {
    match ext.to_lowercase().as_str() {
        "png" => Some(ImageFormat::Png),
        "jpeg" | "jpg" => Some(ImageFormat::Jpeg),
        "gif" => Some(ImageFormat::Gif),
        "ico" => Some(ImageFormat::Ico),
        "bmp" => Some(ImageFormat::Bmp),
        _ => None,
    }
}

fn match_format(ext: &str) -> ImageFormat {
    format_from_str(ext).unwrap_or_else(|| {
        warn!("Unsupported image format. Using PNG instead.");
        ImageFormat::Png
    })
}

fn parse_format(format: &str) -> Result<ImageFormat, String> {
    format_from_str(format)
        .ok_or_else(|| "supported formats are PNG, JPEG, GIF, ICO and BMP".to_string())
}

/// Parse a hex color such as `ff8800` (or `#ff8800`) into `N` channels in the range 0.0-1.0.
fn parse_hex_color<const N: usize>(color: &str) -> Result<[f32; N], String> {
    let hex = color.strip_prefix('#').unwrap_or(color);
    if hex.len() != N * 2 || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(format!("expected {} hex digits", N * 2));
    }
    let mut channels = [0.0; N];
    for (i, channel) in channels.iter_mut().enumerate() {
        let byte = u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).map_err(|e| e.to_string())?;
        *channel = byte as f32 / 255.0;
    }
    Ok(channels)
}

fn parse_rgb(color: &str) -> Result<[f32; 3], String> {
    parse_hex_color(color)
}

fn parse_rgba(color: &str) -> Result<(f32, f32, f32, f32), String> {
    let [r, g, b, a] = parse_hex_color(color)?;
    Ok((r, g, b, a))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_hex_colors() {
        assert_eq!(parse_rgb("ff0080"), Ok([1.0, 0.0, 128.0 / 255.0]));
        assert_eq!(parse_rgb("#FF0080"), Ok([1.0, 0.0, 128.0 / 255.0]));
        assert_eq!(parse_rgba("000000ff"), Ok((0.0, 0.0, 0.0, 1.0)));
    }

    #[test]
    fn rejects_bad_hex_colors() {
        assert!(parse_rgb("fff").is_err());
        assert!(parse_rgb("ff00zz").is_err());
        assert!(parse_rgb("+f+f+f").is_err());
        assert!(parse_rgb("ff00ëë").is_err());
        assert!(parse_rgba("ff0080").is_err());
    }

    #[test]
    fn parses_formats() {
        assert_eq!(parse_format("PNG"), Ok(ImageFormat::Png));
        assert_eq!(parse_format("jpg"), Ok(ImageFormat::Jpeg));
        assert!(parse_format("webp").is_err());
    }
}
