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

/// This method will create a rounded edge around a flat polygon in 3d space.
/// 'base_plate' parameter is the target plate.
/// 'radius' is the radius of the edges.
/// 'degrees' is tells how much of a circle should be created as the rounding.
///     It should be in degrees, not radians.
/// 'ceilheight_mm' is the Z coordinate of the flat polygon in 3D space.
/// 'dir' Is the direction of the round edges: inward or outward
/// 'thr' Throws if a cancel signal was received
/// 'last_offset' An auxiliary output variable to save the last offsetted
///     version of 'base_plate'
/// 'last_height' An auxiliary output to save the last z coordinate of the
/// offsetted base_plate. In other words, where the rounded edges end.
Contour3D round_edges(const ExPolygon& base_plate,
                      double radius_mm,
                      double degrees,
                      double ceilheight_mm,
                      bool dir,
                      ThrowOnCancel thr,
                      ExPolygon& last_offset, double& last_height)
{
    auto ob = base_plate;
    auto ob_prev = ob;
    double wh = ceilheight_mm, wh_prev = wh;
    Contour3D curvedwalls;

    int steps = 30;
    double stepx = radius_mm / steps;
    coord_t s = dir? 1 : -1;
    degrees = std::fmod(degrees, 180);

    // we use sin for x distance because we interpret the angle starting from
    // PI/2
    int tos = degrees < 90?
            int(radius_mm*std::cos(degrees * PI / 180 - PI/2) / stepx) : steps;

    for(int i = 1; i <= tos; ++i) {
        thr();

        ob = base_plate;

        double r2 = radius_mm * radius_mm;
        double xx = i*stepx;
        double x2 = xx*xx;
        double stepy = std::sqrt(r2 - x2);

        offset(ob, s * scaled(xx));
        wh = ceilheight_mm - radius_mm + stepy;

        Contour3D pwalls;
        double prev_x = xx - (i - 1) * stepx;
        pwalls = walls(ob.contour, ob_prev.contour, wh, wh_prev, s*prev_x, thr);

        curvedwalls.merge(pwalls);
        ob_prev = ob;
        wh_prev = wh;
    }

    if(degrees > 90) {
        double tox = radius_mm - radius_mm*std::cos(degrees * PI / 180 - PI/2);
        tos = int(tox / stepx);

        for(int i = 1; i <= tos; ++i) {
            thr();
            ob = base_plate;

            double r2 = radius_mm * radius_mm;
            double xx = radius_mm - i*stepx;
            double x2 = xx*xx;
            double stepy = std::sqrt(r2 - x2);
            offset(ob, s * scaled(xx));
            wh = ceilheight_mm - radius_mm - stepy;

            Contour3D pwalls;
            double prev_x = xx - radius_mm + (i - 1)*stepx;
            pwalls =
                walls(ob_prev.contour, ob.contour, wh_prev, wh, s*prev_x, thr);

            curvedwalls.merge(pwalls);
            ob_prev = ob;
            wh_prev = wh;
        }
    }

    last_offset = std::move(ob);
    last_height = wh;

    return curvedwalls;
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

        // TODO: fails for non convex cluster
//        c = std::accumulate(pp.begin(), pp.end(), Point{0, 0});
//        x(c) /= coord_t(pp.size()); y(c) /= coord_t(pp.size());
        break;
    }
    }

    return c;
}

inline Point centroid(const Polygon& poly) {
    return poly.centroid();
}

/// A fake concave hull that is constructed by connecting separate shapes
/// with explicit bridges. Bridges are generated from each shape's centroid
/// to the center of the "scene" which is the centroid calculated from the shape
/// centroids (a star is created...)
Polygons concave_hull(const Polygons& polys, double maxd_mm, ThrowOnCancel thr)
{
    namespace bgi = boost::geometry::index;
    using SpatElement = std::pair<Point, unsigned>;
    using SpatIndex = bgi::rtree< SpatElement, bgi::rstar<16, 4> >;

    if(polys.empty()) return Polygons();

    const double max_dist = scaled(maxd_mm);

    Polygons punion = union_(polys);   // could be redundant

    if(punion.size() == 1) return punion;

    // We get the centroids of all the islands in the 2D slice
    Points centroids; centroids.reserve(punion.size());
    std::transform(punion.begin(), punion.end(), std::back_inserter(centroids),
                   [](const Polygon& poly) { return centroid(poly); });

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

        if (dist >= max_dist) return Polygon();

        Polygon r;
        auto& ctour = r.points;

        ctour.reserve(3);
        ctour.emplace_back(cc);

        Point d(scaled(nx), scaled(ny));
        ctour.emplace_back(c + Point(-d.y(), d.x()));
        ctour.emplace_back(c + Point(d.y(), -d.x()));
        offset(r, scaled(1.));

        return r;
    });

    // This is unavoidable...
    punion = union_(punion);

    return punion;
}

Polygons concave_hull(const ExPolygons& polys, double maxd, ThrowOnCancel thr)
{
    auto tmp = reserve_vector<Polygon>(polys.size());
    for (auto &ep : polys) tmp.emplace_back(ep.contour);
    return concave_hull(tmp, maxd, thr);
}

void pad_plate(const TriangleMesh &      mesh,
               ExPolygons &              output,
               const std::vector<float> &heights,
               ThrowOnCancel             thrfn)
{
    if (mesh.empty()) return;
    //    m.require_shared_vertices(); // TriangleMeshSlicer needs this
    TriangleMeshSlicer slicer(&mesh);
    
    auto out = reserve_vector<ExPolygons>(heights.size());
    slicer.slice(heights, 0.f, &out, thrfn);

    size_t count = 0;
    for(auto& o : out) count += o.size();

    // Now we have to unify all slice layers which can be an expensive operation
    // so we will try to simplify the polygons
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

void pad_plate(const TriangleMesh &mesh,
                ExPolygons &        output,
                float               h,
                float               layerh,
                ThrowOnCancel       thrfn)
{
    float gnd = float(mesh.bounding_box().min(Z));
    
    std::vector<float> slicegrid = grid(gnd, gnd + h, layerh);
    pad_plate(mesh, output, slicegrid, thrfn);
}

namespace {

// A processed set of values based on the raw pad config parameters. This will
// be used by multiple methods the same way.
struct FineCfg {
    double thickness;
    double wingheight;
    double fullheight;
    double slope;
    double wingdist;
    double bottom_offs;
    double mergedist;
    double radius;
    
    // scaled values
    coord_t s_thickness;
    coord_t s_eradius;
    coord_t s_safety_dist;
    coord_t s_wingdist;
    coord_t s_wingheight;
    coord_t s_bottom_offs;
    coord_t s_waffle_offs;
    
    FineCfg(const PadConfig &cfg) {
        thickness   = cfg.min_wall_thickness_mm;
        wingheight  = cfg.min_wall_height_mm;
        fullheight  = wingheight + thickness;
        slope       = cfg.wall_slope;
        wingdist    = wingheight / std::tan(slope);
        bottom_offs = (thickness + wingheight) / std::tan(slope);
        radius      = cfg.edge_radius_mm;
        
        s_thickness   = scaled(thickness);
        s_eradius     = scaled(cfg.edge_radius_mm);
        s_safety_dist = 2 * s_eradius + coord_t(0.8 * s_thickness);
        s_wingdist    = scaled(wingdist);
        s_wingheight  = scaled(wingheight);
        s_bottom_offs = scaled(bottom_offs);
        
        s_waffle_offs = s_safety_dist + s_wingdist + s_thickness;
        
        mergedist = 2 * (1.8 * thickness + 4 * radius) + cfg.max_merge_dist_mm;
    }
};

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

void offset_waffle_style(Polygons &cntrs, coord_t delta) {
    ClipperLib::Paths paths = Slic3rMultiPoints_to_ClipperPaths(cntrs);
    paths = fast_offset(paths, 2 * delta, ClipperLib::jtRound);
    paths = fast_offset(paths, -delta, ClipperLib::jtRound);
    cntrs = ClipperPaths_to_Slic3rPolygons(paths);
}

template<class P>
Contour3D create_outer_pad(const std::vector<P> &skeleton, const PadConfig &pcfg)
{
    FineCfg C(pcfg);
    auto &thr = pcfg.throw_on_cancel;
    Contour3D ret;
    
    for (const P &pad_part : skeleton) {
        ExPolygon top_poly(pad_part);
        ExPolygon bottom_poly(top_poly);
        offset_single(bottom_poly, -C.s_bottom_offs);
        
        ExPolygon middle_base(top_poly);
        ExPolygon inner_base(top_poly);
        ExPolygon outer_base(top_poly);
        
        if (C.wingheight > 0) { // If pad cavity is requested
            offset_single(inner_base,  -C.s_thickness - C.s_wingdist - C.s_eradius);
            offset_single(middle_base, -C.s_thickness);
            
            top_poly.holes.emplace_back(middle_base.contour);
            auto &tph = top_poly.holes.back().points;
            std::reverse(tph.begin(), tph.end());
        }
        
        ret.merge(walls(outer_base.contour, bottom_poly.contour, 0,
                        -C.fullheight, C.bottom_offs, thr));
        
        if (C.wingheight > 0) {
            // Next is the cavity walls connecting to the top plate's
            // artificially created hole.
            ret.merge(walls(inner_base.contour, middle_base.contour,
                            -C.wingheight, 0, -C.wingdist, thr));
        }
        
        for (auto &h : bottom_poly.holes)
            ret.merge(walls(h, h, 0, -C.fullheight, 0, thr));
        
        ret.merge(triangulate_expolygon_3d(bottom_poly, -C.fullheight, true));
        ret.merge(triangulate_expolygon_3d(top_poly));
        if (C.wingheight > 0)
            ret.merge(triangulate_expolygon_3d(inner_base, -C.wingheight));
    }
    
    return ret;
}

struct AroundPadSkeleton {
    const PadConfig &pcfg;
    FineCfg C;

    AroundPadSkeleton(const ExPolygons &support_contours,
                      const ExPolygons &model_contours,
                      const PadConfig & cfg)
        : pcfg(cfg), C(cfg)
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
        auto &    thr = pcfg.throw_on_cancel;

        for (auto &ep : support_contours)  {
            ExPolygons tmp = offset_ex(ep, C.s_thickness);
            for (auto &t : tmp) m_intersector.add(t);
        }

        // Create a concave hull from just the support contours. We have to
        // include it amongst the support contours as it is a potential part
        // of them. If some model contour does not intersect with any support
        // contour and it is removed from the pad skeleton, then a concave
        // hull is genrated from the support contours and that CAN intersect
        // with some of the model contours. So we consider the concave hull
        // (of supports) as part of the supports.
        Polygons concavehs = concave_hull(support_contours, C.mergedist, thr);
        for (Polygon &p : concavehs) m_intersector.add(ExPolygon(p));
        
        m_outer.reserve(support_contours.size() + model_contours.size());

        // Fill the pad skeleton with the support contours which are certainly
        // part of the pad.
        for (auto &ep : support_contours) m_outer.emplace_back(ep.contour);
        
        ClipperLib::PolyTree ptree = union_pt(model_contours);
        
        auto inner_polys = reserve_vector<ExPolygon>(ptree.Total());
        
        for (ClipperLib::PolyTree::PolyNode *node : ptree.Childs) {
            ExPolygon poly(ClipperPath_to_Slic3rPolygon(node->Contour));
            for (ClipperLib::PolyTree::PolyNode *child : node->Childs) {
                if (child->IsHole()) {
                    poly.holes.emplace_back(
                        ClipperPath_to_Slic3rPolygon(child->Contour));
                    
                    traverse_pt_unordered(child->Childs, &inner_polys);
                }
                else traverse_pt_unordered(child, &inner_polys);
            }
            
            process_skeleton_poly(poly, m_outer);
        }
        
        for (auto &p : inner_polys) process_skeleton_poly(p, m_inner);
    }
    
    ExPolygons outer() const {
        Polygons outer_pad =
            concave_hull(m_outer, C.mergedist, pcfg.throw_on_cancel);
        
        offset_waffle_style(outer_pad, C.s_waffle_offs);
        
        return diff_ex(outer_pad, to_polygons(m_clip));
    }
    
    ExPolygons inner() const {
        return diff_ex(to_polygons(m_inner), to_polygons(m_clip));
    }
    
private:
    
    void process_skeleton_poly(const ExPolygon &poly, ExPolygons &out_skeleton)
    {        
        coord_t s_embed_object_gap = scaled(pcfg.embed_object.object_gap_mm);
        for (const Polygon &hole : poly.holes) {
            ExPolygon holepoly(hole);
            holepoly.contour.reverse();
            
            if (m_intersector.intersects(holepoly)) {
                auto inners = offset_ex(holepoly, -s_embed_object_gap);
                for (ExPolygon &inp : inners) m_inner.emplace_back(inp);
                
            } else m_clip.emplace_back(holepoly);
        }
        
        ExPolygons poly_gap_offs = offset_ex(poly, s_embed_object_gap);
        for (auto &poffs : poly_gap_offs) {
            if (m_intersector.intersects(poffs)) {
                out_skeleton.emplace_back(poffs);
                sla::breakstick_holes(
                    poffs,
                    pcfg.embed_object.object_gap_mm, // padding
                    pcfg.embed_object.stick_stride_mm,
                    pcfg.embed_object.stick_width_mm,
                    pcfg.embed_object.stick_penetration_mm);
                
                m_clip.emplace_back(poffs);
            }
        }
    }
    
    ExPolygons m_outer, m_inner, m_clip;
    Intersector m_intersector;
};

Contour3D pad_around_model(const ExPolygons &support_contours,
                           const ExPolygons &model_contours,
                           const PadConfig  &pcfg)
{    
    Contour3D ret;
    
    AroundPadSkeleton skeleton(support_contours, model_contours, pcfg);
    const FineCfg &C = skeleton.C;
    auto &thr = pcfg.throw_on_cancel;
   
    ret.merge(create_outer_pad(skeleton.outer(), pcfg));
    
    ExPolygons inner_pad_skeleton = skeleton.inner();
    for (const ExPolygon &pad_part : inner_pad_skeleton) {
        ret.merge(walls(pad_part.contour, pad_part.contour, 0, -C.fullheight,
                        C.bottom_offs, thr));

        for (auto &h : pad_part.holes)
            ret.merge(straight_walls(h, 0, -C.fullheight, thr));
        
        ret.merge(triangulate_expolygon_3d(pad_part, -C.fullheight, true));
        ret.merge(triangulate_expolygon_3d(pad_part));
    }

    return ret;   
}

Contour3D pad_below_model(const ExPolygons &support_contours,
                          const ExPolygons &model_contours,
                          const PadConfig &pcfg)
{
    const FineCfg C(pcfg);

    auto pad_skeleton = reserve_vector<Polygon>(support_contours.size() +
                                                model_contours.size());

    for (auto &ep : support_contours) pad_skeleton.emplace_back(ep.contour);
    for (auto &ep : model_contours)   pad_skeleton.emplace_back(ep.contour);

    Contour3D ret;
    auto &    thr = pcfg.throw_on_cancel;
    
    Polygons concavehs = concave_hull(pad_skeleton, C.mergedist, thr);
    
    offset_waffle_style(concavehs, C.s_waffle_offs);
    
    ret.merge(create_outer_pad(concavehs, pcfg));

    return ret;
}

// Dispatch function
Contour3D create_pad(const ExPolygons &support_contours,
                     const ExPolygons &model_contours,
                     const PadConfig & cfg)
{
    auto padfn = cfg.embed_object ? pad_around_model : pad_below_model;
    
    return padfn(support_contours, model_contours, cfg);
}

}

// Interface function
void create_pad(const ExPolygons &sup_contours,
                const ExPolygons &model_contours,
                TriangleMesh &    out,
                const PadConfig & cfg)
{
    out.merge(mesh(create_pad(sup_contours, model_contours, cfg)));
}

}
}
