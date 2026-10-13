use kw_expr::*;
use proptest::prelude::*;
use std::collections::HashMap;

fn scope() -> HashMap<String, Quantity> {
    let mut m = HashMap::new();
    m.insert("width".to_string(), Quantity::length(40.0));
    m.insert("n".to_string(), Quantity::scalar(4.0));
    m.insert("angle".to_string(), Quantity::angle(30f64.to_radians()));
    m
}

fn ev(s: &str) -> Result<Quantity, String> {
    Expr::parse(s)
        .map_err(|e| e.to_string())?
        .eval(&scope())
        .map_err(|e| e.to_string())
}

#[test]
fn arithmetic_and_units() {
    assert_eq!(ev("2 * width + 3 mm").unwrap(), Quantity::length(83.0));
    assert_eq!(ev("1 in").unwrap(), Quantity::length(25.4));
    assert!((ev("2.5 m").unwrap().value - 2500.0).abs() < 1e-9);
    assert_eq!(ev("10 mm / 4").unwrap(), Quantity::length(2.5));
    assert_eq!(ev("width / 2 mm").unwrap().dim, Dim::NONE); // width / 2 / mm? no: (width/2) mm
}

#[test]
fn precedence_and_power() {
    assert_eq!(ev("2 + 3 * 4").unwrap().value, 14.0);
    assert_eq!(ev("-2 ^ 2").unwrap().value, -4.0);
    assert_eq!(ev("2 ^ 3 ^ 2").unwrap().value, 512.0);
    assert_eq!(ev("(2 + 3) * 4").unwrap().value, 20.0);
    let area = ev("width ^ 2").unwrap();
    assert_eq!(area.dim, Dim([2, 0, 0, 0]));
    assert_eq!(ev("sqrt(width ^ 2)").unwrap(), Quantity::length(40.0));
}

#[test]
fn angles_and_functions() {
    let s = ev("sin(30 deg)").unwrap();
    assert!((s.value - 0.5).abs() < 1e-12 && s.dim.is_none());
    assert!((ev("cos(angle)").unwrap().value - 30f64.to_radians().cos()).abs() < 1e-12);
    assert_eq!(ev("atan(1)").unwrap().dim, Dim::ANGLE);
    assert_eq!(
        ev("max(1 mm, 2 mm, 0.5 mm)").unwrap(),
        Quantity::length(2.0)
    );
    assert_eq!(ev("if(n > 3, 5 mm, 2 mm)").unwrap(), Quantity::length(5.0));
    assert_eq!(ev("if(n > 5, 5 mm, 2 mm)").unwrap(), Quantity::length(2.0));
    assert_eq!(ev("floor(2.7)").unwrap().value, 2.0);
    assert!((ev("pi").unwrap().value - std::f64::consts::PI).abs() < 1e-15);
    assert!((ev("90 deg").unwrap().value - std::f64::consts::FRAC_PI_2).abs() < 1e-15);
}

#[test]
fn errors_are_explanatory() {
    let e = ev("width + 3").unwrap_err();
    assert!(e.contains("length") && e.contains("dimensionless"), "{e}");
    let e = ev("3 foo").unwrap_err();
    assert!(e.contains("unknown unit `foo`"), "{e}");
    let e = ev("nothere * 2").unwrap_err();
    assert!(e.contains("unknown variable `nothere`"), "{e}");
    assert!(ev("1 / 0").unwrap_err().contains("division by zero"));
    assert!(ev("sqrt(-1)").unwrap_err().contains("negative"));
    assert!(ev("sqrt(width)").unwrap_err().contains("square root"));
    assert!(ev("2 +").unwrap_err().contains("ends early"));
    assert!(ev("(2").unwrap_err().contains("expected `)`"));
    assert!(ev("").unwrap_err().contains("empty"));
    assert!(ev("sin(width)").unwrap_err().contains("angle"));
    assert!(ev("width ^ 0.5").unwrap_err().contains("integer power"));
    assert!(ev("foo(1)").unwrap_err().contains("unknown function"));
    assert!(ev("1 $ 2").unwrap_err().contains("unexpected character"));
}

#[test]
fn free_vars_are_sorted_and_exclude_units_and_constants() {
    let e = Expr::parse("b * width + 2 mm + pi * 2 deg + sin(width / 1 mm)").unwrap();
    assert_eq!(e.free_vars(), vec!["b".to_string(), "width".to_string()]);
    assert!(is_reserved("mm") && is_reserved("pi") && !is_reserved("width"));
}

#[test]
fn serde_roundtrip_through_json_like_text() {
    // Expr serialises as its source; Param as a tagged value
    let p = Param::expr("2 * width + 3 mm").unwrap();
    assert_eq!(p.free_vars(), vec!["width".to_string()]);
    assert_eq!(p.eval(&scope()).unwrap(), Quantity::length(83.0));
    let q = Param::angle_deg(90.0);
    assert!((q.eval(&scope()).unwrap().value - std::f64::consts::FRAC_PI_2).abs() < 1e-15);
}

#[test]
fn deep_nesting_is_an_error_not_a_stack_overflow() {
    let s = "(".repeat(5000) + "1" + &")".repeat(5000);
    assert!(Expr::parse(&s).is_err());
    let s = "-".repeat(5000) + "1";
    assert!(Expr::parse(&s).is_err());
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(20000))]
    #[test]
    fn arbitrary_text_never_panics(s in "[ -~]{0,60}") {
        if let Ok(e) = Expr::parse(&s) {
            let _ = e.eval(&scope());
            let _ = e.free_vars();
        }
    }

    #[test]
    fn token_soup_never_panics(toks in proptest::collection::vec(
        prop::sample::select(vec!["1", "2.5", "mm", "deg", "(", ")", "+", "-", "*", "/", "^", ",", "sin", "width", "n", "if", "<", ">=", "pi", "e5", "."]),
        0..25)) {
        let s = toks.join(" ");
        if let Ok(e) = Expr::parse(&s) {
            let _ = e.eval(&scope());
        }
    }

    #[test]
    fn linear_expressions_evaluate_like_f64(a in -1e3f64..1e3, b in -1e3f64..1e3, c in 1.0f64..1e3) {
        let s = format!("({a}) * ({b}) + ({a}) / ({c})");
        let v = Expr::parse(&s).unwrap().eval(&scope()).unwrap().value;
        prop_assert!((v - (a * b + a / c)).abs() <= 1e-9 * (1.0 + (a * b).abs()));
    }
}
