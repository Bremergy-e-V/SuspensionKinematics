#include "app/TemplateDraft.h"
#include "app/TemplateEditorDialog.h"
#include "app/TemplateSettingsTab.h"
#include "io/LinkageTemplate.h"
#include "model/HardpointConfig.h"
#include "model/HardpointMirror.h"

#include <QSignalSpy>
#include <QTabWidget>
#include <QTest>

using namespace suspkin;

namespace {

Hardpoint make(const char* name, double x, double y, double z)
{
    Hardpoint point;
    point.name = QLatin1String(name);
    point.coord[0] = x;
    point.coord[1] = y;
    point.coord[2] = z;
    return point;
}

/// The front left corner the solver tests use, with its far side mirrored.
HardpointTable frontAxle()
{
    HardpointTable table;
    table.points = {
        make("F_LCA_IF", 100.0, 200.0, 120.0),     make("F_LCA_IR", -100.0, 200.0, 120.0),
        make("F_LCA_O", 0.0, 560.0, 110.0),        make("F_UCA_IF", 80.0, 230.0, 280.0),
        make("F_UCA_IR", -80.0, 230.0, 280.0),     make("F_UCA_O", 0.0, 540.0, 300.0),
        make("F_TieRod_I", -120.0, 220.0, 150.0),  make("F_TieRod_O", -120.0, 555.0, 145.0),
        make("F_WheelCenter", 0.0, 600.0, 220.0),  make("F_ContactPatch", 0.0, 600.0, 0.0),
    };
    return mirrorHardpoints(table, {}, MirrorSpec{}).table;
}

struct Fixture {
    LinkageTemplate templ = builtinLinkageTemplate();
    HardpointTable table = frontAxle();
    HardpointConfigMap config = inferHardpointConfig(table, templ, MirrorSpec{});
};

int partIndex(const LinkageTemplate& templ, const QString& id)
{
    for (int i = 0; i < int(templ.parts.size()); ++i)
        if (templ.parts[std::size_t(i)].id == id) return i;
    return -1;
}

} // namespace

class TestTemplateEditor : public QObject {
    Q_OBJECT

private slots:
    void theNearSideIsAllThePointTablesShow();
    void relabellingAPartTakesTheRowsThatNamedItAlong();
    void aNewPartIsABodyTheSettingsCanNameAtOnce();
    void theFarSideTakesItsSettingsFromItsSource();
    void aNameIsWrittenForEveryAxle();
    void theBuiltinCornerMovesAsOne();
    void aCornerWithoutItsTieRodSaysWhatIsLoose();
    void theDialogOpensOnItsFourTabsAndAppliesOnlyWhenAsked();
};

void TestTemplateEditor::theNearSideIsAllThePointTablesShow()
{
    Fixture f;
    TemplateDraft draft(f.templ, f.table, f.config, MirrorSpec{});
    NearSideProxy proxy(draft.model());
    QCOMPARE(proxy.rowCount(), 10); // the mirrors are ghosts, and not shown
    proxy.setFarSideShown(true);
    QCOMPARE(proxy.rowCount(), 10); // a ghost is not shown even then

    // A point on the right that is nobody's mirror is the far side proper.
    HardpointTable table = f.table;
    table.points.push_back(make("F_Sensor_R", 0.0, -300.0, 200.0));
    TemplateDraft withSensor(f.templ, table, f.config, MirrorSpec{});
    NearSideProxy near(withSensor.model());
    QCOMPARE(near.rowCount(), 10);
    near.setFarSideShown(true);
    QCOMPARE(near.rowCount(), 11);
}

void TestTemplateEditor::relabellingAPartTakesTheRowsThatNamedItAlong()
{
    Fixture f;
    TemplateDraft draft(f.templ, f.table, f.config, MirrorSpec{});
    const int index = partIndex(draft.linkageTemplate(), QStringLiteral("tieRod"));
    PartTemplate part = draft.linkageTemplate().parts[std::size_t(index)];
    part.label = QStringLiteral("{corner} {side} toe link");
    draft.setPart(index, part);

    QCOMPARE(draft.model()->config().value(QStringLiteral("F_TieRod_O")).part1,
             QStringLiteral("Toe link"));
    QVERIFY(draft.model()->bodyCatalog().contains(QStringLiteral("Toe link")));
    QVERIFY(!draft.model()->bodyCatalog().contains(QStringLiteral("Tie rod")));
    QCOMPARE(draft.renamedBodies().size(), 1);
    QCOMPARE(draft.renamedBodies().front().first, QStringLiteral("Tie rod"));
}

void TestTemplateEditor::aNewPartIsABodyTheSettingsCanNameAtOnce()
{
    Fixture f;
    TemplateDraft draft(f.templ, f.table, f.config, MirrorSpec{});
    QSignalSpy changed(&draft, &TemplateDraft::templateChanged);
    draft.addPart(QStringLiteral("Damper"));
    QCOMPARE(changed.count(), 1);
    const PartTemplate& part = draft.linkageTemplate().parts.back();
    QVERIFY(part.chains.empty());
    QVERIFY(draft.model()->bodyCatalog().contains(partBodyName(part)));

    // It survives a write and a read back, with nothing drawn.
    const LinkageTemplateLoadResult reread = readLinkageTemplate(
        writeLinkageTemplate(draft.linkageTemplate()), QStringLiteral("drafted"));
    QVERIFY2(reread.ok(), qPrintable(reread.error));
    QCOMPARE(reread.templ->parts.size(), draft.linkageTemplate().parts.size());
}

void TestTemplateEditor::theFarSideTakesItsSettingsFromItsSource()
{
    Fixture f;
    TemplateDraft draft(f.templ, f.table, f.config, MirrorSpec{});
    HardpointModel* model = draft.model();
    const int row = model->table().indexOf(QStringLiteral("F_LCA_O"));
    QVERIFY(model->setData(model->index(row, HardpointModel::TypeColumn),
                           int(PointType::Welded), Qt::EditRole));
    const HardpointConfigMap config = draft.configWithMirrors();
    QCOMPARE(config.value(QStringLiteral("F_LCA_O")).type, PointType::Welded);
    QCOMPARE(config.value(mirroredName(QStringLiteral("F_LCA_O"), MirrorSpec{})).type,
             PointType::Welded);
}

void TestTemplateEditor::aNameIsWrittenForEveryAxle()
{
    Fixture f;
    TemplateDraft draft(f.templ, f.table, f.config, MirrorSpec{});
    QCOMPARE(draft.patternOf(QStringLiteral("F_LCA_O")), QStringLiteral("{corner}_LCA_O"));
    QCOMPARE(draft.patternOf(QStringLiteral("Camera")), QStringLiteral("Camera"));
    QVERIFY(draft.pointPatterns().contains(QStringLiteral("{corner}_UCA_O")));
    QVERIFY(!draft.pointPatterns().contains(QStringLiteral("F_UCA_O")));
}

void TestTemplateEditor::theBuiltinCornerMovesAsOne()
{
    Fixture f;
    TemplateDraft draft(f.templ, f.table, f.config, MirrorSpec{});
    const QStringList lines = describeCornerMobility(draft);
    QCOMPARE(lines.size(), 1); // the rear axle has no points
    QVERIFY2(lines.front().contains(QStringLiteral("moves as one corner")),
             qPrintable(lines.front()));
}

void TestTemplateEditor::aCornerWithoutItsTieRodSaysWhatIsLoose()
{
    Fixture f;
    f.config.insert(QStringLiteral("F_TieRod_O"), HardpointConfig{});
    TemplateDraft draft(f.templ, f.table, f.config, MirrorSpec{});
    const QStringList lines = describeCornerMobility(draft);
    QVERIFY2(lines.front().contains(QStringLiteral("Upright")), qPrintable(lines.front()));
    QVERIFY(lines.front().contains(QStringLiteral("roles")));
}

void TestTemplateEditor::theDialogOpensOnItsFourTabsAndAppliesOnlyWhenAsked()
{
    Fixture f;
    TemplateEditorDialog dialog(f.templ, f.table, f.config, MirrorSpec{});
    auto* tabs = dialog.findChild<QTabWidget*>();
    QVERIFY(tabs);
    QCOMPARE(tabs->count(), 4);
    QCOMPARE(tabs->tabText(0), QStringLiteral("Parts"));
    QCOMPARE(tabs->tabText(3), QStringLiteral("Graphics"));

    dialog.setCurrentTab(2);
    QCOMPARE(dialog.currentTab(), 2);
    dialog.setFarSideShown(true);
    QVERIFY(dialog.farSideShown());

    QSignalSpy applied(&dialog, &TemplateEditorDialog::applyRequested);
    dialog.draft().addPart(QStringLiteral("Bracket"));
    QCOMPARE(applied.count(), 0); // editing the draft is not applying it
    dialog.reject();
    QCOMPARE(applied.count(), 0);
}

QTEST_MAIN(TestTemplateEditor)
#include "test_template_editor.moc"
