//! The Sketch workspace: the constraint-solving sketcher, embedded, editing a new sketch or an existing
//! sketch feature of the project. "Finish sketch" writes the sketch back into the feature timeline.

use crate::project::Project;
use kw_expr::Param;
use kw_feature::{FeatureId, FeatureSpec, PlaneSpec, SketchRecipe, SketchSpec};
use kw_sketch::Sketch;
use kw_sketcher::SketcherApp;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NewPlane {
    Xy,
    Yz,
    Xz,
}

impl NewPlane {
    fn spec(self) -> PlaneSpec {
        match self {
            NewPlane::Xy => PlaneSpec::Xy(Param::length(0.0)),
            NewPlane::Yz => PlaneSpec::Yz(Param::length(0.0)),
            NewPlane::Xz => PlaneSpec::Xz(Param::length(0.0)),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Editing {
    Nothing,
    New(PlaneSpecKey),
    Existing(FeatureId),
}

/// A plane a new sketch is placed on (kept comparable for the state machine).
#[derive(Debug, Clone, PartialEq)]
pub struct PlaneSpecKey(pub PlaneSpec);

pub struct SketchWorkspace {
    pub app: SketcherApp,
    pub editing: Editing,
    pub error: String,
    pub plane: NewPlane,
    /// Variable values when an existing sketch was loaded: bindings of variables the user did not change
    /// are kept, those the user changed are dropped (the new value wins).
    loaded_vars: Vec<f64>,
}

impl Default for SketchWorkspace {
    fn default() -> Self {
        let mut app = SketcherApp::default();
        app.embedded = true;
        SketchWorkspace {
            app,
            editing: Editing::Nothing,
            error: String::new(),
            plane: NewPlane::Xy,
            loaded_vars: Vec::new(),
        }
    }
}

impl SketchWorkspace {
    /// Start a sketch on any plane the engine accepts (an origin plane or a face).
    pub fn begin_new_on(&mut self, plane: PlaneSpec) {
        self.app.editor.replace_sketch(Sketch::new());
        self.app.zoom_to_fit();
        self.loaded_vars.clear();
        self.editing = Editing::New(PlaneSpecKey(plane));
        self.error.clear();
    }

    pub fn begin_new(&mut self, plane: NewPlane) {
        self.app.editor.replace_sketch(Sketch::new());
        self.app.zoom_to_fit();
        self.loaded_vars.clear();
        self.editing = Editing::New(PlaneSpecKey(plane.spec()));
        self.error.clear();
    }

    pub fn begin_edit(&mut self, project: &mut Project, id: FeatureId) -> Result<(), String> {
        let doc = project.document();
        let Some(f) = doc.get(id) else {
            return Err("no such feature".into());
        };
        let FeatureSpec::Sketch(s) = &f.spec else {
            return Err(format!("{} is not a sketch", f.name));
        };
        // the evaluated sketch has the parameter bindings applied; the bare recipe does not
        let evaluated = match project.evaluation().output(id).map(|o| &o.product) {
            Some(kw_feature::Product::Sketch(sp)) => Some(sp.sketch.clone()),
            _ => None,
        };
        let sk = match evaluated {
            Some(sk) => sk,
            None => s.recipe.to_sketch().map_err(|e| e.to_string())?.0,
        };
        self.loaded_vars = sk.vars.values.clone();
        self.app.editor.replace_sketch(sk);
        self.app.zoom_to_fit();
        self.editing = Editing::Existing(id);
        self.error.clear();
        Ok(())
    }

    pub fn cancel(&mut self) {
        self.editing = Editing::Nothing;
        self.error.clear();
    }

    /// Write the sketch into the project. Returns the feature that holds it.
    pub fn finish(&mut self, project: &mut Project) -> Option<FeatureId> {
        let mut recipe = SketchRecipe::from_sketch(&self.app.editor.sketch);
        let result = match self.editing.clone() {
            Editing::Nothing => return None,
            Editing::New(PlaneSpecKey(plane)) => {
                let doc = project.document();
                let name = (1..)
                    .map(|n| format!("Sketch {n}"))
                    .find(|n| doc.find_by_name(n).is_none())
                    .expect("a free name");
                project
                    .history
                    .edit(|d| d.add(&name, FeatureSpec::Sketch(SketchSpec { plane, recipe })))
            }
            Editing::Existing(id) => {
                let doc = project.document();
                let Some(FeatureSpec::Sketch(old)) = doc.get(id).map(|f| f.spec.clone()) else {
                    self.error = "the sketch no longer exists".into();
                    return None;
                };
                // keep the parameter bindings of variables that are still there
                let loaded = &self.loaded_vars;
                recipe.bindings = old
                    .recipe
                    .bindings
                    .iter()
                    .filter(|b| {
                        let v = b.var as usize;
                        v < recipe.vars.len()
                            && v < loaded.len()
                            && (recipe.vars[v] - loaded[v]).abs() <= 1e-9 * (1.0 + loaded[v].abs())
                    })
                    .cloned()
                    .collect();
                let plane = old.plane;
                project.history.edit(|d| {
                    d.replace_spec(id, FeatureSpec::Sketch(SketchSpec { plane, recipe }))
                        .map(|_| id)
                })
            }
        };
        match result {
            Ok(id) => {
                self.editing = Editing::Nothing;
                self.error.clear();
                Some(id)
            }
            Err(e) => {
                self.error = e.to_string();
                None
            }
        }
    }
}
