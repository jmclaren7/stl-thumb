//! STEP (ISO 10303-21) loader.
//!
//! STEP files describe exact B-rep geometry, so the shells have to be tessellated before they
//! can be drawn. That is done by the pure-Rust truck CAD kernel. truck does not place the parts
//! of an assembly, so the assembly structure is walked here to find where each shell goes.
//!
//! Everything truck-specific lives in this module behind the `step` feature, so a different
//! kernel can replace it without touching the rest of stl-thumb.

use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::io::Read;

use cgmath::{InnerSpace, Matrix, Matrix3, Matrix4, Point3, SquareMatrix, Transform, Vector3};
use truck_meshalgo::prelude::*;
use truck_stepio::r#in::ruststep::ast::{DataSection, EntityInstance, Name, Parameter, Record};
use truck_stepio::r#in::{ruststep, Table};

use crate::mesh::{Mesh, MeshBuilder};

/// Tessellation tolerance as a fraction of the model's size.
const TOLERANCE: f64 = 0.001;

/// Assemblies deeper than this are treated as a cycle.
const MAX_DEPTH: usize = 64;

pub fn load<R: Read>(mut reader: R) -> Result<Mesh, Box<dyn Error>> {
    let mut bytes = Vec::new();
    reader.read_to_end(&mut bytes)?;
    // STEP is meant to be ASCII, but some exporters write strings in other encodings.
    let text = String::from_utf8_lossy(&bytes);
    let exchange =
        ruststep::parser::parse(&text).map_err(|e| format!("Unable to parse STEP file: {}", e))?;
    let data = exchange
        .data
        .first()
        .ok_or("STEP file has no DATA section")?;

    let instances = Assembly::new(data).shell_instances();
    let table = Table::from_data_section(data);

    // Convert each shell once, even when an assembly places it several times.
    let mut shells = HashMap::new();
    let mut ids: Vec<u64> = instances.iter().map(|i| i.shell.id).collect();
    ids.sort_unstable();
    ids.dedup();
    for id in ids {
        let Some(holder) = table.shell.get(&id) else {
            warn!("STEP shell #{} is not supported", id);
            continue;
        };
        match table.to_compressed_shell(holder) {
            Ok(shell) => {
                shells.insert(id, shell);
            }
            Err(e) => warn!("Unable to read STEP shell #{}: {:?}", id, e),
        }
    }

    // Base the tolerance on the size of the whole model, so small parts are not meshed more
    // finely than they will be drawn.
    let mut bounds = BoundingBox::<Point3<f64>>::new();
    for instance in &instances {
        if let Some(shell) = shells.get(&instance.shell.id) {
            for v in &shell.vertices {
                bounds.push(instance.transform.transform_point(*v));
            }
        }
    }
    let size = if bounds.is_empty() {
        0.0
    } else {
        bounds.diameter()
    };

    let mut meshes = HashMap::new();
    for (id, shell) in &shells {
        let tolerance = if size > 0.0 {
            size * TOLERANCE
        } else {
            // No vertices to measure (a sphere, for example), so measure a rough mesh instead.
            let rough = shell.robust_triangulation(0.01).to_polygon();
            rough.bounding_box().diameter().max(f64::EPSILON) * TOLERANCE
        };
        let meshed = shell.robust_triangulation(tolerance);
        let failed = meshed.faces.iter().filter(|f| f.surface.is_none()).count();
        if failed > 0 {
            warn!("Unable to mesh {} faces of STEP shell #{}", failed, id);
        }
        meshes.insert(*id, meshed.to_polygon());
    }

    let mut builder = MeshBuilder::new();
    for instance in &instances {
        if let Some(polygon) = meshes.get(&instance.shell.id) {
            add_polygon(&mut builder, polygon, instance);
        }
    }
    builder.finish("STEP")
}

fn add_polygon(builder: &mut MeshBuilder, polygon: &PolygonMesh, instance: &Instance) {
    let transform = &instance.transform;
    let positions = polygon.positions();
    let normals = polygon.normals();
    let linear = Matrix3::from_cols(
        transform.x.truncate(),
        transform.y.truncate(),
        transform.z.truncate(),
    );
    let mut normal_matrix = linear.invert().map(|m| m.transpose()).unwrap_or(linear);
    if instance.shell.reversed {
        normal_matrix = -normal_matrix;
    }
    // A mirroring transform turns counter-clockwise triangles clockwise, and a reversed shell
    // has them inside out, so swap them back.
    let order = if (linear.determinant() < 0.0) != instance.shell.reversed {
        [0, 2, 1]
    } else {
        [0, 1, 2]
    };

    for triangle in polygon.faces().triangle_iter() {
        let vertices = order.map(|i| {
            let p = transform.transform_point(positions[triangle[i].pos]);
            [p.x as f32, p.y as f32, p.z as f32]
        });
        let normals = order
            .iter()
            .map(|&i| {
                let n = normals.get(triangle[i].nor?)?;
                let n = (normal_matrix * n).normalize();
                Some([n.x as f32, n.y as f32, n.z as f32])
            })
            .collect::<Option<Vec<_>>>()
            .filter(|n| n.iter().flatten().all(|c| c.is_finite()))
            .map(|n| [n[0], n[1], n[2]]);
        builder.add_triangle(vertices, normals);
    }
}

/// A shell as referenced from a solid or surface model, possibly with its faces reversed.
#[derive(Clone, Copy)]
struct ShellRef {
    id: u64,
    reversed: bool,
}

/// A shell placed in the model.
struct Instance {
    shell: ShellRef,
    transform: Matrix4<f64>,
}

/// The parts of the STEP entity graph that say where each shell is placed.
struct Assembly<'a> {
    entities: HashMap<u64, &'a EntityInstance>,
}

impl<'a> Assembly<'a> {
    fn new(data: &'a DataSection) -> Self {
        let entities = data
            .entities
            .iter()
            .map(|e| match e {
                EntityInstance::Simple { id, .. } | EntityInstance::Complex { id, .. } => (*id, e),
            })
            .collect();
        Assembly { entities }
    }

    /// Every shell to draw, with the transform that places it in the model.
    fn shell_instances(&self) -> Vec<Instance> {
        // Representations joined by a plain relationship share a coordinate system.
        // Relationships with a transformation, and mapped items, place a child inside a parent.
        let mut links: HashMap<u64, Vec<u64>> = HashMap::new();
        let mut children: HashMap<u64, Vec<(u64, Matrix4<f64>)>> = HashMap::new();
        let mut shells: HashMap<u64, Vec<ShellRef>> = HashMap::new();
        let mut is_child = HashSet::new();
        let mut known_shells = HashSet::new();
        let parents = self.assembly_parents();

        for (&id, entity) in &self.entities {
            let records = records(entity);
            // In a complex entity the representations are in the REPRESENTATION_RELATIONSHIP
            // record, and SHAPE_REPRESENTATION_RELATIONSHIP() is empty.
            if let Some(r) = records.iter().find(|r| {
                (r.name == "REPRESENTATION_RELATIONSHIP"
                    || r.name == "SHAPE_REPRESENTATION_RELATIONSHIP")
                    && arguments(r).len() >= 4
            }) {
                let (Some(rep_1), Some(rep_2)) = (reference(r, 2), reference(r, 3)) else {
                    continue;
                };
                let transformation = records
                    .iter()
                    .find(|r| r.name == "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
                    .and_then(|r| reference(r, 0));
                match transformation {
                    None => {
                        links.entry(rep_1).or_default().push(rep_2);
                        links.entry(rep_2).or_default().push(rep_1);
                    }
                    Some(transformation) => {
                        // The standard puts the child in rep_1, but some exporters (SolidWorks,
                        // for one) swap them. The product structure says which is which.
                        if !parents.contains_key(&id) {
                            debug!("No product structure for STEP relationship #{}", id);
                        }
                        let parent_is_rep_1 = parents.get(&id) == Some(&rep_1);
                        let (parent, child) = if parent_is_rep_1 {
                            (rep_1, rep_2)
                        } else {
                            (rep_2, rep_1)
                        };
                        let transform = self.transformation(transformation, parent_is_rep_1);
                        children.entry(parent).or_default().push((child, transform));
                        is_child.insert(child);
                    }
                }
            } else if let Some(r) = records.iter().find(|r| is_shape_representation(&r.name)) {
                for item in references(r, 1) {
                    let Some(item_record) = self.record(item) else {
                        continue;
                    };
                    match item_record.name.as_str() {
                        "MANIFOLD_SOLID_BREP" | "FACETED_BREP" | "BREP_WITH_VOIDS" => {
                            if let Some(shell) = reference(item_record, 1) {
                                let shell = self.shell(shell, &mut known_shells);
                                shells.entry(id).or_default().push(shell);
                            }
                            // Voids are hidden inside the solid, so they are never drawn.
                            if item_record.name == "BREP_WITH_VOIDS" {
                                for void in references(item_record, 2) {
                                    self.shell(void, &mut known_shells);
                                }
                            }
                        }
                        "SHELL_BASED_SURFACE_MODEL" => {
                            for shell in references(item_record, 1) {
                                let shell = self.shell(shell, &mut known_shells);
                                shells.entry(id).or_default().push(shell);
                            }
                        }
                        "MAPPED_ITEM" => {
                            if let Some((child, transform)) = self.mapped_item(item_record) {
                                children.entry(id).or_default().push((child, transform));
                                is_child.insert(child);
                            }
                        }
                        _ => (),
                    }
                }
            }
        }

        // Group representations that share a coordinate system, then walk down from the groups
        // that nothing places.
        let mut group_of: HashMap<u64, usize> = HashMap::new();
        let mut groups: Vec<Vec<u64>> = Vec::new();
        let mut reps: Vec<u64> = links
            .keys()
            .chain(children.keys())
            .chain(shells.keys())
            .chain(is_child.iter())
            .copied()
            .collect();
        reps.sort_unstable();
        reps.dedup();
        for &rep in &reps {
            if group_of.contains_key(&rep) {
                continue;
            }
            let mut group = Vec::new();
            let mut stack = vec![rep];
            group_of.insert(rep, groups.len());
            while let Some(r) = stack.pop() {
                group.push(r);
                for &next in links.get(&r).into_iter().flatten() {
                    if let std::collections::hash_map::Entry::Vacant(e) = group_of.entry(next) {
                        e.insert(groups.len());
                        stack.push(next);
                    }
                }
            }
            groups.push(group);
        }

        let mut instances = Vec::new();
        for (index, group) in groups.iter().enumerate() {
            if !group.iter().any(|r| is_child.contains(r)) {
                let walk = Walk {
                    groups: &groups,
                    group_of: &group_of,
                    children: &children,
                    shells: &shells,
                };
                walk.visit(index, Matrix4::identity(), 0, &mut instances);
            }
        }

        // Draw any shell the assembly structure does not account for where it is.
        let mut loose: Vec<u64> = self
            .entities
            .iter()
            .filter(|(id, e)| {
                !known_shells.contains(*id)
                    && records(e)
                        .iter()
                        .any(|r| r.name == "CLOSED_SHELL" || r.name == "OPEN_SHELL")
            })
            .map(|(id, _)| *id)
            .collect();
        loose.sort_unstable();
        instances.extend(loose.into_iter().map(|id| Instance {
            shell: ShellRef {
                id,
                reversed: false,
            },
            transform: Matrix4::identity(),
        }));
        instances
    }

    /// Resolve a shell reference to the shell truck reads and whether it is reversed, marking
    /// both as accounted for. ORIENTED_CLOSED_SHELL and ORIENTED_OPEN_SHELL are
    /// (name, *, shell, orientation) wrappers, which truck keeps apart from its shells.
    fn shell(&self, id: u64, known_shells: &mut HashSet<u64>) -> ShellRef {
        known_shells.insert(id);
        let Some(r) = self
            .record(id)
            .filter(|r| r.name == "ORIENTED_CLOSED_SHELL" || r.name == "ORIENTED_OPEN_SHELL")
        else {
            return ShellRef {
                id,
                reversed: false,
            };
        };
        let Some(shell) = reference(r, 2) else {
            return ShellRef {
                id,
                reversed: false,
            };
        };
        known_shells.insert(shell);
        let reversed = matches!(
            arguments(r).get(3),
            Some(Parameter::Enumeration(e)) if e.trim_matches('.') == "F"
        );
        ShellRef {
            id: shell,
            reversed,
        }
    }

    /// Map each transformed representation relationship to the representation of the parent
    /// assembly, by following its product structure:
    /// CONTEXT_DEPENDENT_SHAPE_REPRESENTATION -> PRODUCT_DEFINITION_SHAPE ->
    /// NEXT_ASSEMBLY_USAGE_OCCURRENCE -> parent PRODUCT_DEFINITION -> its representations.
    fn assembly_parents(&self) -> HashMap<u64, u64> {
        let mut definition_reps: HashMap<u64, Vec<u64>> = HashMap::new();
        for entity in self.entities.values() {
            for r in records(entity) {
                if r.name == "SHAPE_DEFINITION_REPRESENTATION" {
                    let definition = reference(r, 0)
                        .and_then(|pds| self.record(pds))
                        .and_then(|pds| reference(pds, 2));
                    if let (Some(definition), Some(rep)) = (definition, reference(r, 1)) {
                        definition_reps.entry(definition).or_default().push(rep);
                    }
                }
            }
        }

        let mut parents = HashMap::new();
        for entity in self.entities.values() {
            for r in records(entity) {
                if r.name != "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION" {
                    continue;
                }
                let (Some(relationship), Some(pds)) = (reference(r, 0), reference(r, 1)) else {
                    continue;
                };
                let parent_definition = self
                    .record(pds)
                    .and_then(|pds| reference(pds, 2))
                    .and_then(|nauo| self.record(nauo))
                    .and_then(|nauo| reference(nauo, 3));
                let reps = self
                    .entities
                    .get(&relationship)
                    .map(|e| records(e))
                    .unwrap_or_default()
                    .into_iter()
                    .find(|r| r.name == "REPRESENTATION_RELATIONSHIP")
                    .map(|r| [reference(r, 2), reference(r, 3)]);
                if let (Some(parent_definition), Some(reps)) = (parent_definition, reps) {
                    let parent_reps = &definition_reps
                        .get(&parent_definition)
                        .cloned()
                        .unwrap_or_default();
                    if let Some(parent) =
                        reps.into_iter().flatten().find(|r| parent_reps.contains(r))
                    {
                        parents.insert(relationship, parent);
                    }
                }
            }
        }
        parents
    }

    /// The transform from the child's coordinates to the parent's for an
    /// ITEM_DEFINED_TRANSFORMATION, whose first item is in rep_1 and second item is in rep_2.
    fn transformation(&self, id: u64, parent_is_rep_1: bool) -> Matrix4<f64> {
        let Some(r) = self
            .record(id)
            .filter(|r| r.name == "ITEM_DEFINED_TRANSFORMATION")
        else {
            warn!("Unsupported STEP assembly transformation #{}", id);
            return Matrix4::identity();
        };
        let (parent_item, child_item) = if parent_is_rep_1 { (2, 3) } else { (3, 2) };
        let parent = reference(r, parent_item).and_then(|p| self.placement(p));
        let child = reference(r, child_item).and_then(|p| self.placement(p));
        match (parent, child) {
            (Some(parent), Some(child)) => parent * child.invert().unwrap_or(Matrix4::identity()),
            _ => Matrix4::identity(),
        }
    }

    /// MAPPED_ITEM(name, REPRESENTATION_MAP(origin, representation), target) places a copy of
    /// a representation's origin at the target.
    fn mapped_item(&self, r: &Record) -> Option<(u64, Matrix4<f64>)> {
        let map = self.record(reference(r, 1)?)?;
        let child = reference(map, 1)?;
        let origin = reference(map, 0).and_then(|p| self.placement(p));
        let target = reference(r, 2).and_then(|p| self.placement(p));
        let transform = match (target, origin) {
            (Some(target), Some(origin)) => target * origin.invert()?,
            (Some(target), None) => target,
            _ => Matrix4::identity(),
        };
        Some((child, transform))
    }

    /// AXIS2_PLACEMENT_3D(name, location, axis, ref_direction) as a matrix from the
    /// placement's coordinates to its parent's.
    fn placement(&self, id: u64) -> Option<Matrix4<f64>> {
        let r = self.record(id).filter(|r| r.name == "AXIS2_PLACEMENT_3D")?;
        let location = self.coordinates(reference(r, 1)?)?;
        let z = reference(r, 2)
            .and_then(|d| self.coordinates(d))
            .map(|d| Vector3::from(d).normalize())
            .unwrap_or(Vector3::unit_z());
        let x = reference(r, 3)
            .and_then(|d| self.coordinates(d))
            .map(Vector3::from)
            .unwrap_or(Vector3::unit_x());
        // Make x perpendicular to z, as the standard does.
        let mut x = x - z * x.dot(z);
        if x.magnitude2() < 1e-12 {
            x = if z.x.abs() < 0.9 {
                Vector3::unit_x()
            } else {
                Vector3::unit_y()
            };
            x = x - z * x.dot(z);
        }
        let x = x.normalize();
        let y = z.cross(x);
        Some(Matrix4::from_cols(
            x.extend(0.0),
            y.extend(0.0),
            z.extend(0.0),
            Vector3::from(location).extend(1.0),
        ))
    }

    /// The coordinates of a CARTESIAN_POINT or DIRECTION.
    fn coordinates(&self, id: u64) -> Option<[f64; 3]> {
        let r = self.record(id)?;
        let Some(Parameter::List(values)) = arguments(r).get(1) else {
            return None;
        };
        let mut coordinates = [0.0; 3];
        for (c, value) in coordinates.iter_mut().zip(values) {
            *c = match value {
                Parameter::Real(v) => *v,
                Parameter::Integer(v) => *v as f64,
                _ => return None,
            };
        }
        Some(coordinates)
    }

    fn record(&self, id: u64) -> Option<&'a Record> {
        match self.entities.get(&id)? {
            EntityInstance::Simple { record, .. } => Some(record),
            EntityInstance::Complex { .. } => None,
        }
    }
}

struct Walk<'a> {
    groups: &'a [Vec<u64>],
    group_of: &'a HashMap<u64, usize>,
    children: &'a HashMap<u64, Vec<(u64, Matrix4<f64>)>>,
    shells: &'a HashMap<u64, Vec<ShellRef>>,
}

impl Walk<'_> {
    fn visit(
        &self,
        group: usize,
        transform: Matrix4<f64>,
        depth: usize,
        instances: &mut Vec<Instance>,
    ) {
        if depth > MAX_DEPTH {
            warn!("STEP assembly is nested too deeply or forms a cycle");
            return;
        }
        for rep in &self.groups[group] {
            for &shell in self.shells.get(rep).into_iter().flatten() {
                instances.push(Instance { shell, transform });
            }
            for (child, child_transform) in self.children.get(rep).into_iter().flatten() {
                if let Some(&child_group) = self.group_of.get(child) {
                    self.visit(
                        child_group,
                        transform * child_transform,
                        depth + 1,
                        instances,
                    );
                }
            }
        }
    }
}

fn is_shape_representation(name: &str) -> bool {
    name.ends_with("SHAPE_REPRESENTATION") && name != "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"
}

fn records(entity: &EntityInstance) -> Vec<&Record> {
    match entity {
        EntityInstance::Simple { record, .. } => vec![record],
        EntityInstance::Complex { subsuper, .. } => subsuper.0.iter().collect(),
    }
}

fn arguments(r: &Record) -> &[Parameter] {
    match &r.parameter {
        Parameter::List(arguments) => arguments,
        _ => &[],
    }
}

fn reference(r: &Record, index: usize) -> Option<u64> {
    match arguments(r).get(index)? {
        Parameter::Ref(Name::Entity(id)) => Some(*id),
        _ => None,
    }
}

fn references(r: &Record, index: usize) -> Vec<u64> {
    match arguments(r).get(index) {
        Some(Parameter::List(items)) => items
            .iter()
            .filter_map(|item| match item {
                Parameter::Ref(Name::Entity(id)) => Some(*id),
                _ => None,
            })
            .collect(),
        _ => Vec::new(),
    }
}
