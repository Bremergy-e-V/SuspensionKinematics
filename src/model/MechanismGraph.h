#pragma once

#include "geom/Vec3.h"
#include "model/HardpointConfig.h"
#include "model/Linkage.h"
#include "model/Mechanism.h"

#include <QString>
#include <QStringList>

#include <vector>

namespace suspkin {

/// One body of a corner, as the solve sees it: a name and whether it keeps its
/// shape. The chassis is always body @ref MechanismGraph::kGround.
struct GraphBody {
    QString name;
    bool compressible = false;
};

/// One hardpoint, with the two bodies that meet there by index into
/// @ref MechanismGraph::bodies, or @ref MechanismGraph::kNoBody.
struct GraphPoint {
    QString name;
    Vec3 design;
    PointType type = PointType::Unassigned;
    int body1 = -1;
    int body2 = -1;
};

/// What one corner of one side is made of, for the solver: the bodies, and every
/// point of the corner with what joins what there.
///
/// It is plain data and says nothing about how it is solved. Two things build
/// one: the configuration table the user fills in (@ref configGraph), and the
/// mechanism block's roles on their own (@ref roleGraph), which is what a corner
/// with no configuration -- or a caller that never had one -- is solved from.
struct MechanismGraph {
    static constexpr int kGround = 0;
    static constexpr int kNoBody = -1;

    std::vector<GraphBody> bodies; ///< kGround first
    std::vector<GraphPoint> points;

    bool isEmpty() const { return points.empty(); }
    int indexOfPoint(const QString& name) const;
    int indexOfBody(const QString& name) const;
    /// The body called @p name, added when there is none yet.
    int bodyFor(const QString& name, bool compressible = false);
};

/// Every point name one corner and one side of @p templ draws or gives a role,
/// named as @p table names them: the per-corner parts' chains and the mechanism's
/// roles, with {corner} filled in, mirrored for the far side and read through
/// former names. That is what decides which of the configuration's points belong
/// to this corner -- the configuration itself is keyed by name over the whole car.
QStringList cornerPointNames(const LinkageTemplate& templ, const QString& cornerToken,
                             bool mirrored, const MirrorSpec& mirror, const HardpointTable& table);

/// The graph @p config describes for the points called @p names: each point that
/// is in @p table and has a type joins the bodies it names. A body is
/// compressible when the template part it is named after is.
MechanismGraph configGraph(const QStringList& names, const HardpointConfigMap& config,
                           const LinkageTemplate& templ, const HardpointTable& table);

/// The graph the roles of @p mechanism imply on their own -- @p mechanism named
/// for one corner and side, as AxleSolver hands it over. It is the double
/// wishbone the closed-form solve always assumed: the wishbones about their
/// pivots, the upright on the three outer joints, the tie rod, the pushrod on
/// the arm its mount names, the rocker, the damper and the bar.
MechanismGraph roleGraph(const MechanismTemplate& mechanism, const HardpointTable& table);

} // namespace suspkin
