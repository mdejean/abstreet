use std::collections::HashSet;
use std::fmt;

use anyhow::Result;
use serde::{Deserialize, Serialize};

use crate::{Distance, GPSBounds, Line, PolyLine, Polygon, Pt2D};

/// Maybe a misnomer, but like a PolyLine, but closed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ring {
    // first equals last
    pts: Vec<Pt2D>,
}

impl Ring {
    pub fn new(pts: Vec<Pt2D>) -> Result<Ring> {
        if pts.len() < 3 {
            bail!("Can't make a ring with < 3 points");
        }
        if pts[0] != *pts.last().unwrap() {
            bail!("Can't make a ring with mismatching first/last points");
        }

        if pts.windows(2).any(|pair| pair[0] == pair[1]) {
            bail!("Ring has ~dupe adjacent pts");
        }

        let result = Ring { pts };

        let mut seen_pts = HashSet::new();
        for pt in result.pts.iter().skip(1) {
            seen_pts.insert(pt.to_hashable());
        }
        if seen_pts.len() != result.pts.len() - 1 {
            bail!("Ring has repeat non-adjacent points");
        }

        Ok(result)
    }
    pub fn must_new(pts: Vec<Pt2D>) -> Ring {
        Ring::new(pts).unwrap()
    }

    /// Draws the ring with some thickness, with half of it straddling the interor of the ring, and
    /// half on the outside.
    pub fn to_outline(&self, thickness: Distance) -> Polygon {
        // TODO Has a weird corner. Use the polygon offset thing instead?
        PolyLine::unchecked_new(self.pts.clone()).make_polygons(thickness)
    }

    pub fn into_polygon(self) -> Polygon {
        Polygon::with_holes(self, Vec::new())
    }

    pub fn points(&self) -> &Vec<Pt2D> {
        &self.pts
    }
    pub fn into_points(self) -> Vec<Pt2D> {
        self.pts
    }

    /// Be careful with the order of results. Hits on an earlier line segment of other show up
    /// first, but if the ring hits a line segment at multiple points, who knows. Dedupes.
    pub fn all_intersections(&self, other: &PolyLine) -> Vec<Pt2D> {
        let mut hits = Vec::new();
        let mut seen = HashSet::new();
        for l1 in other.lines() {
            for l2 in self
                .pts
                .windows(2)
                .map(|pair| Line::must_new(pair[0], pair[1]))
            {
                if let Some(pt) = l1.intersection(&l2) {
                    if !seen.contains(&pt.to_hashable()) {
                        hits.push(pt);
                        seen.insert(pt.to_hashable());
                    }
                }
            }
        }
        hits
    }

    /// Assuming both points are somewhere along the ring, trace along the ring between the two
    /// points. There are two ways to do this, so return both. The result is oriented from `pt1` to
    /// `pt2`. If the input points are the same, returns `None`.
    pub(crate) fn get_both_slices_between(
        &self,
        pt1: Pt2D,
        pt2: Pt2D,
    ) -> Option<(PolyLine, PolyLine)> {
        if pt1 == pt2 {
            return None;
        }
        let pl = PolyLine::unchecked_new(self.pts.clone());

        let mut dist1 = pl.dist_along_of_point(pt1)?.0;
        let mut dist2 = pl.dist_along_of_point(pt2)?.0;
        let mut swapped = false;
        if dist1 > dist2 {
            std::mem::swap(&mut dist1, &mut dist2);
            swapped = true;
        }

        let mut candidate1 = pl.maybe_exact_slice(dist1, dist2).ok()?;
        let mut candidate2 = pl
            .maybe_exact_slice(dist2, pl.length())
            .ok()?
            .must_extend(pl.maybe_exact_slice(Distance::ZERO, dist1).ok()?);

        // Orient both results from pt1 to pt2
        if swapped {
            candidate1 = candidate1.reversed();
        } else {
            candidate2 = candidate2.reversed();
        }
        // TODO Breaking in blockfinder...
        /*if candidate1.first_pt() != pt1 {
            println!("wat? pt1 {}, pt2 {}, candidate1 {}, swapped {}", pt1, pt2, candidate1, swapped);
        }
        assert_eq!(candidate1.first_pt(), pt1);
        assert_eq!(candidate2.first_pt(), pt1);
        assert_eq!(candidate1.last_pt(), pt2);
        assert_eq!(candidate2.last_pt(), pt2);*/

        Some((candidate1, candidate2))
    }

    /// Assuming both points are somewhere along the ring, return the points in between the two, by
    /// tracing along the ring in the longer or shorter direction (depending on `longer`). If both
    /// points are the same, returns `None`.  The result is oriented from `pt1` to `pt2`.
    pub fn get_slice_between(&self, pt1: Pt2D, pt2: Pt2D, longer: bool) -> Option<PolyLine> {
        let (candidate1, candidate2) = self.get_both_slices_between(pt1, pt2)?;
        let slice = if longer == (candidate1.length() > candidate2.length()) {
            candidate1
        } else {
            candidate2
        };
        Some(slice)
    }

    pub fn get_shorter_slice_between(&self, pt1: Pt2D, pt2: Pt2D) -> Option<PolyLine> {
        self.get_slice_between(pt1, pt2, false)
    }

    /// Extract all PolyLines and Rings. Doesn't handle crazy double loops and stuff.
    pub fn split_points(pts: &[Pt2D]) -> Result<(Vec<PolyLine>, Vec<Ring>)> {
        let mut seen = HashSet::new();
        let mut intersections = HashSet::new();
        for pt in pts {
            let pt = pt.to_hashable();
            if seen.contains(&pt) {
                intersections.insert(pt);
            } else {
                seen.insert(pt);
            }
        }
        intersections.insert(pts[0].to_hashable());
        intersections.insert(pts.last().unwrap().to_hashable());

        let mut polylines = Vec::new();
        let mut rings = Vec::new();
        let mut current = Vec::new();
        for pt in pts.iter().cloned() {
            current.push(pt);
            if intersections.contains(&pt.to_hashable()) && current.len() > 1 {
                if current[0] == pt && current.len() >= 3 {
                    rings.push(Ring::new(current.drain(..).collect())?);
                } else {
                    polylines.push(PolyLine::new(current.drain(..).collect())?);
                }
                current.push(pt);
            }
        }
        Ok((polylines, rings))
    }

    pub fn contains_pt(&self, pt: Pt2D) -> bool {
        PolyLine::unchecked_new(self.pts.clone())
            .dist_along_of_point(pt)
            .is_some()
    }

    /// Produces a GeoJSON polygon, optionally mapping the world-space points back to GPS.
    pub fn to_geojson(&self, gps: Option<&GPSBounds>) -> geojson::Geometry {
        let mut pts = Vec::new();
        if let Some(gps) = gps {
            for pt in gps.convert_back(&self.pts) {
                pts.push(vec![pt.x(), pt.y()]);
            }
        } else {
            for pt in &self.pts {
                pts.push(vec![pt.x(), pt.y()]);
            }
        }
        geojson::Geometry::new(geojson::Value::Polygon(vec![pts]))
    }

    /// Translates the ring by a fixed offset.
    pub fn translate(mut self, dx: f64, dy: f64) -> Ring {
        for pt in &mut self.pts {
            *pt = pt.offset(dx, dy);
        }
        self
    }
}

impl fmt::Display for Ring {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "Ring::new(vec![")?;
        for pt in &self.pts {
            writeln!(f, "  Pt2D::new({}, {}),", pt.x(), pt.y())?;
        }
        write!(f, "])")
    }
}

impl From<Ring> for geo::LineString<f64> {
    fn from(ring: Ring) -> Self {
        let coords = ring
            .pts
            .into_iter()
            .map(geo::Coordinate::from)
            .collect::<Vec<_>>();
        Self(coords)
    }
}

impl From<geo::LineString<f64>> for Ring {
    fn from(line_string: geo::LineString<f64>) -> Self {
        // Dedupe adjacent points. Only needed for results from concave hull.
        let mut pts: Vec<Pt2D> = line_string.0.into_iter().map(Pt2D::from).collect();
        pts.dedup();
        Self::must_new(pts)
    }
}
