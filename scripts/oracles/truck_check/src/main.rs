//! usage: truck_check <dir-with-manifest.jsonl>
//! Parses each Kerfwright STEP export with the independent `truck` kernel's STEP reader and checks
//! that the file converts to shells with the face count recorded in the manifest.
use std::collections::BTreeMap;
use truck_stepio::r#in::Table;

fn main() {
    let dir = std::env::args().nth(1).expect("dir");
    let manifest = std::fs::read_to_string(format!("{dir}/manifest.jsonl")).expect("manifest");
    let (mut total, mut ok) = (0, 0);
    let mut failures: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for line in manifest.lines() {
        let file = field_str(line, "file");
        let faces: usize = field_num(line, "faces") as usize;
        total += 1;
        let text = std::fs::read_to_string(format!("{dir}/{file}")).expect("read");
        let res = std::panic::catch_unwind(|| -> Result<usize, String> {
            let table = Table::from_step(&text).ok_or("parse failed")?;
            let mut n = 0;
            for shell in table.shell.values() {
                let cs = table
                    .to_compressed_shell(shell)
                    .map_err(|e| format!("{e:?}"))?;
                n += cs.faces.len();
            }
            if table.shell.is_empty() {
                return Err("no shells".into());
            }
            Ok(n)
        });
        let kind = file.rsplit_once('_').map(|x| x.0).unwrap_or(&file).to_string();
        match res {
            Ok(Ok(n)) if n == faces => ok += 1,
            Ok(Ok(n)) => failures.entry(kind).or_default().push(format!("{file}: {n} faces, expected {faces}")),
            Ok(Err(e)) => failures.entry(kind).or_default().push(format!("{file}: {}", e.chars().take(120).collect::<String>())),
            Err(_) => failures.entry(kind).or_default().push(format!("{file}: panic in truck")),
        }
    }
    println!("files: {total}  parsed+converted with matching face count: {ok}  failed: {}", total - ok);
    for (k, v) in failures {
        println!("  {k}: {} failed, e.g. {}", v.len(), v[0]);
    }
}

fn field_str(line: &str, key: &str) -> String {
    let pat = format!("\"{key}\":\"");
    let i = line.find(&pat).unwrap() + pat.len();
    line[i..i + line[i..].find('"').unwrap()].to_string()
}
fn field_num(line: &str, key: &str) -> f64 {
    let pat = format!("\"{key}\":");
    let i = line.find(&pat).unwrap() + pat.len();
    let rest = &line[i..];
    rest[..rest.find([',', '}']).unwrap()].parse().unwrap()
}
