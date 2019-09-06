#include "SLABasePool.hpp"
#include "SLABoilerPlate.hpp"
#include "SLASpatIndex.hpp"

#include "boost/log/trivial.hpp"
#include "SLABoostAdapter.hpp"
#include "ClipperUtils.hpp"
#include "Tesselate.hpp"
#include "MTUtils.hpp"

// For debugging:
// #include <fstream>
// #include <libnest2d/tools/benchmark.h>
// #include "SVG.hpp"

namespace Slic3r { namespace sla {

namespace {

/// This function will return a triangulation of a sheet connecting an upper
/// and a lower plate given as input polygons. It will not triangulate the
/// plates themselves only the sheet. The caller has to specify the lower and
/// upper z levels in world coordinates as well as the offset difference
/// between the sheets. If the lower_z_mm is higher than upper_z_mm or the
/// offset difference is negative, the resulting triangle orientation will be
/// reversed.
///
/// IMPORTANT: This is not a universal triangulation algorithm. It assumes
/// that the lower and upper polygons are offsetted versions of the same
/// original polygon. In general, it assumes that one of the polygons is
/// completely inside the other. The offset difference is the reference
/// distance from the inner polygon's perimeter to the outer polygon's
/// perimeter. The real distance will be variable as the clipper offset has
/// different strategies (rounding, etc...). This algorithm should have
/// O(2n + 3m) complexity where n is the number of upper vertices and m is the
/// number of lower vertices.
Contour3D walls(const Polygon& lower, const Polygon& upper,
                double lower_z_mm, double upper_z_mm,
                double offset_difference_mm, ThrowOnCancel thr)
{
    Contour3D ret;

    if(upper.points.size() < 3 || lower.size() < 3) return ret;

    // The concept of the algorithm is relatively simple. It will try to find
    // the closest vertices from the upper and the lower polygon and use those
    // as starting points. Then it will create the triangles sequentially using
    // an edge from the upper polygon and a vertex from the lower or vice versa,
    // depending on the resulting triangle's quality.
    // The quality is measured by a scalar value. So far it looks like it is
    // enough to derive it from the slope of the triangle's two edges connecting
    // the upper and the lower part. A reference slope is calculated from the
    // height and the offset difference.

    // Offset in the index array for the ceiling
    const auto offs = upper.points.size();

    // Shorthand for the vertex arrays
    auto& upts = upper.points, &lpts = lower.points;
    auto& rpts = ret.points; auto& ind = ret.indices;

    // If the Z levels are flipped, or the offset difference is negative, we
    // will interpret that as the triangles normals should be inverted.
    bool inverted = upper_z_mm < lower_z_mm || offset_difference_mm < 0;

    // Copy the points into the mesh, convert them from 2D to 3D
    rpts.reserve(upts.size() + lpts.size());
    ind.reserve(2 * upts.size() + 2 * lpts.size());
    for (auto &p : upts)
        rpts.emplace_back(unscaled(p.x()), unscaled(p.y()), upper_z_mm);
    for (auto &p : lpts)
        rpts.emplace_back(unscaled(p.x()), unscaled(p.y()), lower_z_mm);

    // Create pointing indices into vertex arrays. u-upper, l-lower
    size_t uidx = 0, lidx = offs, unextidx = 1, lnextidx = offs + 1;

    // Simple squared distance calculation.
    auto distfn = [](const Vec3d& p1, const Vec3d& p2) {
        auto p = p1 - p2; return p.transpose() * p;
    };

    // We need to find the closest point on lower polygon to the first point on
    // the upper polygon. These will be our starting points.
    double distmin = std::numeric_limits<double>::max();
    for(size_t l = lidx; l < rpts.size(); ++l) {
        thr();
        double d = distfn(rpts[l], rpts[uidx]);
        if(d < distmin) { lidx = l; distmin = d; }
    }

    // Set up lnextidx to be ahead of lidx in cyclic mode
    lnextidx = lidx + 1;
    if(lnextidx == rpts.size()) lnextidx = offs;

    // This will be the flip switch to toggle between upper and lower triangle
    // creation mode
    enum class Proceed {
        UPPER, // A segment from the upper polygon and one vertex from the lower
        LOWER  // A segment from the lower polygon and one vertex from the upper
    } proceed = Proceed::UPPER;

    // Flags to help evaluating loop termination.
    bool ustarted = false, lstarted = false;

    // The variables for the fitness values, one for the actual and one for the
    // previous.
    double current_fit = 0, prev_fit = 0;

    // Every triangle of the wall has two edges connecting the upper plate with
    // the lower plate. From the length of these two edges and the zdiff we
    // can calculate the momentary squared offset distance at a particular
    // position on the wall. The average of the differences from the reference
    // (squared) offset distance will give us the driving fitness value.
    const double offsdiff2 = std::pow(offset_difference_mm, 2);
    const double zdiff2 = std::pow(upper_z_mm - lower_z_mm, 2);

    // Mark the current vertex iterator positions. If the iterators return to
    // the same position, the loop can be terminated.
    size_t uendidx = uidx, lendidx = lidx;

    do { thr();  // check throw if canceled

        prev_fit = current_fit;

        switch(proceed) {   // proceed depending on the current state
        case Proceed::UPPER:
            if(!ustarted || uidx != uendidx) { // there are vertices remaining
                // Get the 3D vertices in order
                const Vec3d& p_up1 = rpts[uidx];
                const Vec3d& p_low = rpts[lidx];
                const Vec3d& p_up2 = rpts[unextidx];

                // Calculate fitness: the average of the two connecting edges
                double a = offsdiff2 - (distfn(p_up1, p_low) - zdiff2);
                double b = offsdiff2 - (distfn(p_up2, p_low) - zdiff2);
                current_fit = (std::abs(a) + std::abs(b)) / 2;

                if(current_fit > prev_fit) { // fit is worse than previously
                    proceed = Proceed::LOWER;
                } else {    // good to go, create the triangle
                    inverted
                        ? ind.emplace_back(int(unextidx), int(lidx), int(uidx))
                        : ind.emplace_back(int(uidx), int(lidx), int(unextidx));

                    // Increment the iterators, rotate if necessary
                    ++uidx; ++unextidx;
                    if(unextidx == offs) unextidx = 0;
                    if(uidx == offs) uidx = 0;

                    ustarted = true;    // mark the movement of the iterators
                    // so that the comparison to uendidx can be made correctly
                }
            } else proceed = Proceed::LOWER;

            break;
        case Proceed::LOWER:
            // Mode with lower segment, upper vertex. Same structure:
            if(!lstarted || lidx != lendidx) {
                const Vec3d& p_low1 = rpts[lidx];
                const Vec3d& p_low2 = rpts[lnextidx];
                const Vec3d& p_up   = rpts[uidx];

                double a = offsdiff2 - (distfn(p_up, p_low1) - zdiff2);
                double b = offsdiff2 - (distfn(p_up, p_low2) - zdiff2);
                current_fit = (std::abs(a) + std::abs(b)) / 2;

                if(current_fit > prev_fit) {
                    proceed = Proceed::UPPER;
                } else {
                    inverted
                        ? ind.emplace_back(int(uidx), int(lnextidx), int(lidx))
                        : ind.emplace_back(int(lidx), int(lnextidx), int(uidx));

                    ++lidx; ++lnextidx;
                    if(lnextidx == rpts.size()) lnextidx = offs;
                    if(lidx == rpts.size()) lidx = offs;

                    lstarted = true;
                }
            } else proceed = Proceed::UPPER;

            break;
        } // end of switch
    } while(!ustarted || !lstarted || uidx != uendidx || lidx != lendidx);

    return ret;
}

Contour3D inline straight_walls(const Polygon &plate,
                                double         lower_z_mm,
                                double         upper_z_mm,
                                ThrowOnCancel  thr)
{
    return walls(plate, plate, lower_z_mm, upper_z_mm, 0 /*offset_diff*/, thr);
}

inline Point centroid(Points& pp) {
    Point c;
    switch(pp.size()) {
    case 0: break;
    case 1: c = pp.front(); break;
    case 2: c = (pp[0] + pp[1]) / 2; break;
    default: {
        auto MAX = std::numeric_limits<Point::coord_type>::max();
        auto MIN = std::numeric_limits<Point::coord_type>::min();
        Point min = {MAX, MAX}, max = {MIN, MIN};

        for(auto& p : pp) {
            if(p(0) < min(0)) min(0) = p(0);
            if(p(1) < min(1)) min(1) = p(1);
            if(p(0) > max(0)) max(0) = p(0);
            if(p(1) > max(1)) max(1) = p(1);
        }
        c(0) = min(0) + (max(0) - min(0)) / 2;
        c(1) = min(1) + (max(1) - min(1)) / 2;
        break;
    }
    }

    return c;
}

inline Point centroid(const ExPolygon& poly) {
    return poly.contour.centroid();
}

// Function to cut tiny connector cavities for a given polygon. The input poly
// will be offsetted by "padding" and small rectangle shaped cavities will be
// inserted along the perimeter in every "stride" distance. The stick rectangles
// will have a with about "stick_width". The input dimensions are in world
// measure, not the scaled clipper units.
void breakstick_holes(ExPolygon& poly,
                      double padding,
                      double stride,
                      double stick_width,
                      double penetration)
{
    // SVG svg("bridgestick_plate.svg");
    // svg.draw(poly);
    
    auto transf = [stick_width, penetration, padding, stride](Points &pts) {
        // The connector stick will be a small rectangle with dimensions
        // stick_width x (penetration + padding) to have some penetration
        // into the input polygon.
        
        Points out;
        out.reserve(2 * pts.size()); // output polygon points
        
        // stick bottom and right edge dimensions
        double sbottom = scaled(stick_width);
        double sright  = scaled(penetration + padding);
        
        // scaled stride distance
        double sstride = scaled(stride);
        double t       = 0;
        
        // process pairs of vertices as an edge, start with the last and
        // first point
        for (size_t i = pts.size() - 1, j = 0; j < pts.size(); i = j, ++j) {
            // Get vertices and the direction vectors
            const Point &a = pts[i], &b = pts[j];
            Vec2d        dir = b.cast<double>() - a.cast<double>();
            double       nrm = dir.norm();
            dir /= nrm;
            Vec2d dirp(-dir(Y), dir(X));
            
            // Insert start point
            out.emplace_back(a);
            
            // dodge the start point, do not make sticks on the joins
            while (t < sbottom) t += sbottom;
            double tend = nrm - sbottom;
            
            while (t < tend) { // insert the stick on the polygon perimeter
                
                // calculate the stick rectangle vertices and insert them
                // into the output.
                Point p1 = a + (t * dir).cast<coord_t>();
                Point p2 = p1 + (sright * dirp).cast<coord_t>();
                Point p3 = p2 + (sbottom * dir).cast<coord_t>();
                Point p4 = p3 + (sright * -dirp).cast<coord_t>();
                out.insert(out.end(), {p1, p2, p3, p4});
                
                // continue along the perimeter
                t += sstride;
            }
            
            t = t - nrm;
            
            // Insert edge endpoint
            out.emplace_back(b);
        }
        
        // move the new points
        out.shrink_to_fit();
        pts.swap(out);
    };
    
    if(stride > 0.0 && stick_width > 0.0 && padding > 0.0) {
        transf(poly.contour.points);
        for (auto &h : poly.holes) transf(h.points);
    }
    
    // svg.draw(poly);
    // svg.Close();
}

/// A fake concave hull that is constructed by connecting separate shapes
/// with explicit bridges. Bridges are generated from each shape's centroid
/// to the center of the "scene" which is the centroid calculated from the shape
/// centroids (a star is created...)
ExPolygons concave_hull(const ExPolygons& polys, double maxd_mm, ThrowOnCancel thr)
{
    namespace bgi = boost::geometry::index;
    using SpatElement = std::pair<Point, unsigned>;
    using SpatIndex = bgi::rtree< SpatElement, bgi::rstar<16, 4> >;
    
    if(polys.empty()) return {};

    const double max_dist = scaled(maxd_mm);

    ExPolygons punion = union_ex(polys);   // could be redundant

    if(punion.size() == 1) return punion;

    // We get the centroids of all the islands in the 2D slice
    Points centroids; centroids.reserve(punion.size());
    std::transform(punion.begin(), punion.end(), std::back_inserter(centroids),
                   [](const ExPolygon& poly) { return centroid(poly); });

    SpatIndex ctrindex;
    unsigned  idx = 0;
    for(const Point &ct : centroids) ctrindex.insert(std::make_pair(ct, idx++));

    // Centroid of the centroids of islands. This is where the additional
    // connector sticks are routed.
    Point cc = centroid(centroids);

    punion.reserve(punion.size() + centroids.size());

    idx = 0;
    std::transform(centroids.begin(), centroids.end(),
                   std::back_inserter(punion),
                   [&centroids, &ctrindex, cc, max_dist, &idx, thr]
                   (const Point& c)
    {
        thr();
        double dx = c.x() - cc.x(), dy = c.y() - cc.y();
        double l = std::sqrt(dx * dx + dy * dy);
        double nx = dx / l, ny = dy / l;

        Point& ct = centroids[idx];

        std::vector<SpatElement> result;
        ctrindex.query(bgi::nearest(ct, 2), std::back_inserter(result));

        double dist = max_dist;
        for (const SpatElement &el : result)
            if (el.second != idx) {
                dist = Line(el.first, ct).length();
                break;
            }

        idx++;

        if (dist >= max_dist) return ExPolygon();

        ExPolygon r;
        auto& ctour = r.contour.points;

        ctour.reserve(3);
        ctour.emplace_back(cc);

        Point d(scaled(nx), scaled(ny));
        ctour.emplace_back(c + Point(-d.y(), d.x()));
        ctour.emplace_back(c + Point(d.y(), -d.x()));
        offset(r, scaled(1.));

        return r;
    });

    // This is unavoidable...
    punion = union_ex(punion);

    return punion;
}

double get_merge_distance(const PadConfig &c)
{
    return 2. * (1.8 * c.wall_thickness_mm) + c.max_merge_dist_mm;
}

coord_t get_waffle_offset(const PadConfig &c)
{
    return scaled(c.brim_size_mm + c.wing_distance() + c.wall_thickness_mm);  
}

// A helper class for storing polygons and maintaining a spatial index of their
// bounding boxes.
class Intersector {
    BoxIndex       m_index;
    ExPolygons     m_polys;
    
public:
    
    // Add a new polygon to the index
    void add(const ExPolygon &ep) {
        m_polys.emplace_back(ep);
        m_index.insert(BoundingBox{ep}, unsigned(m_index.size()));
    }
    
    // Check an arbitrary polygon for intersection with the indexed polygons
    bool intersects(const ExPolygon &poly)
    {
        // Create a suitable query bounding box.
        auto bb = poly.contour.bounding_box();
        
        std::vector<BoxIndexEl> qres = m_index.query(bb, BoxIndex::qtIntersects);
        
        // Now check intersections on the actual polygons (not just the boxes)
        bool is_overlap = false;
        auto qit        = qres.begin();
        while (!is_overlap && qit != qres.end())
            is_overlap = is_overlap || poly.overlaps(m_polys[(qit++)->second]);
        
        return is_overlap;
    }  
};

ClipperLib::Paths fast_offset(const ClipperLib::Paths &paths,
                              coord_t                  delta,
                              ClipperLib::JoinType     jointype)
{
    using ClipperLib::ClipperOffset;
    using ClipperLib::etClosedPolygon;
    using ClipperLib::Paths;
    using ClipperLib::Path;
    
    ClipperOffset offs;
    offs.ArcTolerance = scaled<double>(0.01);
    
    for (auto &p : paths)
        // If the input is not at least a triangle, we can not do this algorithm
        if(p.size() < 3) {
            BOOST_LOG_TRIVIAL(error) << "Invalid geometry for offsetting!";
            return {};
        }
        
    offs.AddPaths(paths, jointype, etClosedPolygon);

    Paths result; 
    offs.Execute(result, static_cast<double>(delta));
    
    return result;
}

template<class...Args> void offset_single(ExPolygon &poly, Args...args) {
    ExPolygons tmp = offset_ex(poly, args...);
    
    assert(tmp.size() == 1);
    if (tmp.empty()) return;
    
    poly = tmp.front();
}

void offset_waffle_style(ExPolygons &cntrs, coord_t delta) {
    ClipperLib::Paths paths = Slic3rMultiPoints_to_ClipperPaths(cntrs);
    paths = fast_offset(paths, 2 * delta, ClipperLib::jtRound);
    paths = fast_offset(paths, -delta, ClipperLib::jtRound);
    cntrs = ClipperPaths_to_Slic3rExPolygons(paths);
}

// Part of the pad configuration that is used for 3D geometry generation
struct PadConfig3D {
    double thickness, height, wing_height, slope;

    explicit PadConfig3D(const PadConfig &cfg2d)
        : thickness{cfg2d.wall_thickness_mm}
        , height{cfg2d.full_height()}
        , wing_height{cfg2d.wall_height_mm}
        , slope{cfg2d.wall_slope}
    {}

    inline double bottom_offset() const
    {
        return (thickness + wing_height) / std::tan(slope);
    }
};

// Outer part of the skeleton is used to generate the waffled edges of the pad.
// Inner parts will not be waffled or offsetted. Inner parts are only used if
// pad is generated around the object and correspond to holes and inner polygons
// in the model blueprint.
struct PadSkeleton {
    ExPolygons inner, outer;
};

Contour3D create_outer_pad_geometry(const ExPolygons & skeleton,
                                    const PadConfig3D &cfg,
                                    ThrowOnCancel      thr)
{
    Contour3D ret;
    
    double wing_distance = cfg.wing_height / std::tan(cfg.slope);
    
    for (const ExPolygon &pad_part : skeleton) {
        ExPolygon top_poly(pad_part);
        ExPolygon bottom_poly(top_poly);
        offset_single(bottom_poly, -scaled(cfg.bottom_offset()));
        
        ExPolygon middle_base(top_poly);
        ExPolygon inner_base(top_poly);
        ExPolygon outer_base(top_poly);
        
        if (cfg.wing_height > 0) { // If pad cavity is requested
            offset_single(inner_base,  -scaled(cfg.thickness + wing_distance));
            offset_single(middle_base, -scaled(cfg.thickness));
            
            ExPolygons pdiff = diff_ex(top_poly, middle_base.contour);
            assert(pdiff.size() == 1);
            if (!pdiff.empty()) top_poly = pdiff.front();
        }
        
        double z_min = -cfg.height, z_max = 0;
        ret.merge(walls(outer_base.contour, bottom_poly.contour, z_max, z_min,
                        cfg.bottom_offset(), thr));

        for (auto &h : bottom_poly.holes)
            ret.merge(straight_walls(h, z_max, z_min, thr));
        
        ret.merge(triangulate_expolygon_3d(bottom_poly, z_min, NORMALS_DOWN));
        ret.merge(triangulate_expolygon_3d(top_poly, NORMALS_UP));
        
        if (cfg.wing_height > 0) { // triangulate the cavity and its walls
            z_min = -cfg.wing_height;
            z_max = 0;
            double offset_difference = -wing_distance;
            ret.merge(walls(inner_base.contour, middle_base.contour, z_min,
                            z_max, offset_difference, thr));
            
            ret.merge(triangulate_expolygon_3d(inner_base, z_min, NORMALS_UP));
        }    
    }
    
    return ret;
}

Contour3D create_inner_pad_geometry(const ExPolygons & skeleton,
                                    const PadConfig3D &cfg,
                                    ThrowOnCancel      thr)
{
    Contour3D ret;
    
    double z_max = 0., z_min = -cfg.height;
    for (const ExPolygon &pad_part : skeleton) {
        ret.merge(walls(pad_part.contour, pad_part.contour, z_max, z_min,
                        cfg.bottom_offset(), thr));
        
        for (auto &h : pad_part.holes)
            ret.merge(straight_walls(h, z_max, z_min, thr));
        
        ret.merge(triangulate_expolygon_3d(pad_part, z_min, NORMALS_DOWN));
        ret.merge(triangulate_expolygon_3d(pad_part));
    }
    
    return ret;
}

PadSkeleton divide_blueprint(const ExPolygons &bp)
{
    ClipperLib::PolyTree ptree = union_pt(bp);
    
    PadSkeleton ret;
    ret.inner.reserve(size_t(ptree.Total()));
    ret.outer.reserve(size_t(ptree.Total()));
    
    for (ClipperLib::PolyTree::PolyNode *node : ptree.Childs) {
        ExPolygon poly(ClipperPath_to_Slic3rPolygon(node->Contour));
        for (ClipperLib::PolyTree::PolyNode *child : node->Childs) {
            if (child->IsHole()) {
                poly.holes.emplace_back(
                    ClipperPath_to_Slic3rPolygon(child->Contour));
                
                traverse_pt_unordered(child->Childs, &ret.inner);
            }
            else traverse_pt_unordered(child, &ret.inner);
        }
        
        ret.outer.emplace_back(poly);
    }
    
    return ret;
}

Contour3D create_pad_geometry(const PadSkeleton &skelet, const PadConfig &cfg)
{
    PadConfig3D cfg3d(cfg);
    auto &thr = cfg.throw_on_cancel;
    return create_outer_pad_geometry(skelet.outer, cfg3d, thr)
           .merge(create_inner_pad_geometry(skelet.inner, cfg3d, thr));
}

template<class _Intersector>
class _AroundPadSkeleton : public PadSkeleton
{
    PadConfig::EmbedObject m_cfg;
    ExPolygons             m_clip;
    _Intersector           m_intersector;

public:
    _AroundPadSkeleton(const ExPolygons &support_blueprint,
                       const ExPolygons &model_blueprint,
                       const PadConfig & cfg)
        : m_cfg{cfg.embed_object}
    {
        // We need to merge the support and the model contours in a special
        // way in which the model contours have to be substracted from the
        // support contours. The pad has to have a hole in which the model can
        // fit perfectly (thus the substraction -- diff_ex). Also, the pad has
        // to be eliminated from areas where there is no need for a pad, due
        // to missing supports.

        // m_intersector is a spatial index used here to be able to efficiently
        // find intersections with the model contours. Model contours which do
        // not have any intersection with the support contours have to be
        // removed. (e.g. a hole in the model contour which does not host any
        // supports or supports are not present outside the model only in some
        // of its holes)

        Contour3D ret;
        auto &    thr = cfg.throw_on_cancel;
        
        coord_t s_thickness = scaled(cfg.wall_thickness_mm);
        for (auto &ep : support_blueprint)  {
            ExPolygons tmp = offset_ex(ep, s_thickness);
            for (auto &t : tmp) m_intersector.add(t);
        }

        // Create a concave hull from just the support contours. We have to
        // include it amongst the support contours as it is a potential part
        // of them. If some model contour does not intersect with any support
        // contour and it is removed from the pad skeleton, then a concave
        // hull is genrated from the support contours and that CAN intersect
        // with some of the model contours. So we consider the concave hull
        // (of supports) as part of the supports.
        ExPolygons concavehs = concave_hull(support_blueprint,
                                            get_merge_distance(cfg), thr);
        
        for (ExPolygon &p : concavehs) m_intersector.add(p);
        
        outer.reserve(support_blueprint.size() + model_blueprint.size());

        // Fill the pad skeleton with the support contours which are certainly
        // part of the pad.
        for (auto &ep : support_blueprint) outer.emplace_back(ep.contour);
        
        PadSkeleton divided = divide_blueprint(model_blueprint);
        for (auto &p : divided.outer) process_skeleton_poly(p, outer);
        for (auto &p : divided.inner) process_skeleton_poly(p, inner);
        
        ExPolygons outer_pad = concave_hull(outer, get_merge_distance(cfg), thr);
        
        offset_waffle_style(outer_pad, get_waffle_offset(cfg));
        
        outer = diff_ex(outer_pad, m_clip);
        inner = diff_ex(inner, m_clip);
            
        m_clip = {};
    }

private:
    
    void process_skeleton_poly_holes(ExPolygon &poly)
    {
        auto it = poly.holes.begin();
        while (it != poly.holes.end()) {
            ExPolygon holepoly(*it);
            holepoly.contour.reverse();
            auto inners = offset_ex(holepoly, -scaled(m_cfg.object_gap_mm));
            for (ExPolygon &inp : inners) inner.emplace_back(inp);
            
            if (m_intersector.intersects(holepoly)) ++it;
            else {
                m_clip.emplace_back(holepoly);
                it = poly.holes.erase(it);
            }
        }
    }
    
    void process_skeleton_poly(const ExPolygon &poly, ExPolygons &output)
    {
        coord_t s_embed_object_gap = scaled(m_cfg.object_gap_mm);
        ExPolygon polycpy = poly;
        
        process_skeleton_poly_holes(polycpy);
        
        ExPolygons poly_gap_offs = offset_ex(polycpy, s_embed_object_gap);
        for (auto &poffs : poly_gap_offs) {
            if (m_intersector.intersects(poffs)) output.emplace_back(poffs);
            
            sla::breakstick_holes(
                poffs,
                m_cfg.object_gap_mm,
                m_cfg.stick_stride_mm,
                m_cfg.stick_width_mm,
                m_cfg.stick_penetration_mm);
            
            m_clip.emplace_back(poffs);
        }
    }
};

struct DummyIntersector
{
    inline void add(const ExPolygon &) {}
    inline bool intersects(const ExPolygon &) { return true; }
};

using AroundPadSkeleton = _AroundPadSkeleton<Intersector>;
using BrimPadSkeleton   = _AroundPadSkeleton<DummyIntersector>;

class BelowPadSkeleton : public PadSkeleton
{
public:
    BelowPadSkeleton(const ExPolygons &support_blueprint,
                     const ExPolygons &model_blueprint,
                     const PadConfig & cfg)
    {
        outer.reserve(support_blueprint.size() + model_blueprint.size());

        for (auto &ep : support_blueprint) outer.emplace_back(ep.contour);
        for (auto &ep : model_blueprint) outer.emplace_back(ep.contour);

        outer = concave_hull(outer, get_merge_distance(cfg), cfg.throw_on_cancel);
        offset_waffle_style(outer, get_waffle_offset(cfg));
    }
};

Contour3D create_pad_geometry(const ExPolygons &support_blueprint,
                              const ExPolygons &model_blueprint,
                              const PadConfig & cfg)
{
    PadSkeleton skelet;
    
    if (cfg.embed_object.enabled) {
        if (cfg.embed_object.force_brim)
            skelet = BrimPadSkeleton(support_blueprint, model_blueprint, cfg);
        else
            skelet = AroundPadSkeleton(support_blueprint, model_blueprint, cfg);
    }
    else
        skelet = BelowPadSkeleton(support_blueprint, model_blueprint, cfg);
    
    return create_pad_geometry(skelet, cfg);
}

} // namespace

void pad_blueprint(const TriangleMesh &      mesh,
                   ExPolygons &              output,
                   const std::vector<float> &heights,
                   ThrowOnCancel             thrfn)
{
    if (mesh.empty()) return;
    TriangleMeshSlicer slicer(&mesh);
    
    auto out = reserve_vector<ExPolygons>(heights.size());
    slicer.slice(heights, 0.f, &out, thrfn);
    
    size_t count = 0;
    for(auto& o : out) count += o.size();
    
    // Unification is expensive, a simplify also speeds up the pad generation
    auto tmp = reserve_vector<ExPolygon>(count);
    for(ExPolygons& o : out)
        for(ExPolygon& e : o) {
            auto&& exss = e.simplify(scaled<double>(0.1));
            for(ExPolygon& ep : exss) tmp.emplace_back(std::move(ep));
        }
    
    ExPolygons utmp = union_ex(tmp);
    
    for(auto& o : utmp) {
        auto&& smp = o.simplify(scaled<double>(0.1));
        output.insert(output.end(), smp.begin(), smp.end());
    }
}

void pad_blueprint(const TriangleMesh &mesh,
                   ExPolygons &        output,
                   float               h,
                   float               layerh,
                   ThrowOnCancel       thrfn)
{
    float gnd = float(mesh.bounding_box().min(Z));
    
    std::vector<float> slicegrid = grid(gnd, gnd + h, layerh);
    pad_blueprint(mesh, output, slicegrid, thrfn);
}

void create_pad(const ExPolygons &sup_contours,
                const ExPolygons &model_blueprint,
                TriangleMesh &    out,
                const PadConfig & cfg)
{
    out.merge(mesh(create_pad_geometry(sup_contours, model_blueprint, cfg)));
}

}} // namespace Slic3r::sla
