#pragma once

#include "model/GeomSolve.h"
#include "model/MechanismGraph.h"

#include <QString>
#include <QStringList>

#include <optional>
#include <vector>

namespace suspkin {

/// Where every body of a corner is at one instant, as the motion it made from
/// the design position. One entry per rigid group of the compiled graph; a sweep
/// hands the last one back in as the place to start the next solve from, which
/// is what keeps the solve on the branch the suspension is actually on.
struct GraphState {
    std::vector<Rigid> motions;
};

/// What the mechanism leaves free, for whoever has to say whether it describes
/// a suspension at all.
struct GraphMobility {
    /// Motions left once the driven body is held: zero for a corner that the
    /// lower wishbone's angle and the rack fully decide.
    int freeWithDriveHeld = 0;
    /// Motions there are with nothing held: one for a corner that moves, none
    /// for a mechanism that is locked solid.
    int freeWithDriveLoose = 0;
    /// The bodies the left-over motions move, by name.
    QStringList looseBodies;
    /// Points no moving body carries, by name: they would stand still.
    QStringList unposedPoints;

    bool isDetermined() const
    {
        return freeWithDriveHeld == 0 && freeWithDriveLoose > 0 && unposedPoints.isEmpty();
    }
};

/// What is wrong with a linkage whose mobility is @p mobility, in words -- the
/// points nothing carries, the motions left free and the bodies they move, or a
/// linkage locked solid. Empty when it moves as one corner.
QString describeMobilityProblem(const GraphMobility& mobility);

/// The solve of any linkage a @ref MechanismGraph describes.
///
/// Compiling does the bookkeeping once: bodies joined rigidly become one group,
/// a link with nothing but a joint at each end becomes a distance between them,
/// and every joint becomes a handful of equations in the groups' motions. A
/// solve is then Gauss-Newton on those motions from wherever the mechanism was
/// last, with one body's motion given -- the lower wishbone turned by the arm
/// angle -- and one chassis point moved by the rack.
class GraphSolver {
public:
    /// @p drivenBody is the body whose motion every solve is given. Nothing,
    /// with @p error set, when it is not a moving body of @p graph.
    static std::optional<GraphSolver> compile(const MechanismGraph& graph, int drivenBody,
                                              QString* error);

    /// The motions that hold every joint together with the driven body moved by
    /// @p driven and the point @p rackPoint (or none, -1) moved by @p rackOffset.
    /// @p previous, when given, is where to start from. False, with @p error set,
    /// when the joints cannot all be held -- the mechanism does not assemble.
    bool solve(const Rigid& driven, int rackPoint, const Vec3& rackOffset,
               const GraphState* previous, GraphState* out, QString* error) const;

    /// Where point @p point of the graph is in @p state, carried by the body that
    /// carries it. @p rackPoint and @p rackOffset as given to the solve.
    Vec3 positionOf(int point, const GraphState& state, int rackPoint = -1,
                    const Vec3& rackOffset = Vec3()) const;
    /// Whether anything carries point @p point at all.
    bool isPosed(int point) const;
    /// Whether point @p point moves -- is carried by something other than the
    /// chassis, or is the rack's.
    bool moves(int point, int rackPoint) const;
    /// The motion of body @p body in @p state; nothing moves a body the solve
    /// keeps no motion for, such as a link it reduced to a length.
    Rigid motionOf(int body, const GraphState& state) const;

    /// How free the mechanism is at its design position.
    GraphMobility mobility() const;

    const MechanismGraph& graph() const { return m_graph; }

private:
    GraphSolver() = default;

    enum class Kind { Coincide, Distance, Collinear };
    /// One joint's equations. Coincide holds @ref a, carried by @ref groupA, on
    /// @ref groupB; Distance holds points @ref a and @ref b @ref length apart;
    /// Collinear keeps @ref b on the line through @ref a along @ref direction,
    /// turned with @ref groupA.
    struct Constraint {
        Kind kind = Kind::Coincide;
        int a = -1;
        int b = -1;
        int groupA = -1;
        int groupB = -1;
        double length = 0.0;
        Vec3 direction;
    };

    /// Which groups' motions are being solved for, and where each one's six
    /// unknowns start: a turn, scaled to millimetres, then a shift.
    struct Unknowns {
        std::vector<int> first; ///< per group; -1 for one that is not solved for
        int count = 0;
    };

    void groupBodies();
    void reduceLinks();
    void findCarriers();
    void buildConstraints();
    void findCentroids();
    void findLineGroups();
    Unknowns unknowns(bool includeDriven) const;
    int rows() const;

    /// Where @p point is when carried by @p group in @p state.
    Vec3 carriedBy(int point, int group, const GraphState& state, int rackPoint,
                   const Vec3& rackOffset) const;
    void residual(const GraphState& state, int rackPoint, const Vec3& rackOffset,
                  std::vector<double>* r) const;
    void jacobian(const GraphState& state, int rackPoint, const Vec3& rackOffset,
                  const Unknowns& unknowns, std::vector<double>* j) const;
    GraphState stepped(const GraphState& state, const Unknowns& unknowns,
                       const std::vector<double>& step) const;
    int freeMotions(bool includeDriven, QStringList* looseBodies) const;

    MechanismGraph m_graph;
    std::vector<int> m_groupOfBody; ///< per body; -1 for a compressible body
    int m_groupCount = 0;
    int m_groundGroup = 0;
    int m_drivenGroup = -1;
    std::vector<bool> m_reduced;    ///< per group: a link the solve keeps as a length
    std::vector<int> m_carrier;     ///< per point: the group that carries it, or -1
    std::vector<Vec3> m_centroid;   ///< per group, at design: what a turn is about
    /// Per group: the line every point it touches lies on, when they all do --
    /// a strut-like body, whose spin about that line moves nothing -- or zero.
    std::vector<Vec3> m_lineAxis;
    std::vector<Constraint> m_constraints;
    Unknowns m_unknowns;
};

} // namespace suspkin
