#pragma once
#include <QtQml/qqmlregistration.h>
#include <algorithm>
#include <cmath>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QGuiApplication>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QQuickPaintedItem>
#include <QRectF>
#include <QString>
#include <QVariantList>
#include <QtMath>
#include <qqmlintegration.h>


// ============================================================
// BeamCompassItem —— 波束罗盘（任务页主区）：方位 = 角度（正北朝上、顺时针），高度 = 半径（天顶在圆心）。
// QML 里叫 `BeamCompass`。
//
// 画两样东西：
//   · **计划预览** —— 各束位置、行进顺序（数字）、够不着的束（红 ×）、路径
//     （循环四轴按有无滑环画轮末续转 / 轮末回程，蛇形标"往复"）；
//   · **运行进度** —— 当前正在走的那一束画成实心 + 光晕。
// 数据由任务页算好绑进来，本类不认识设备，也不认识任务。
//
// **为什么是 QQuickPaintedItem 而不是 QSGGeometryNode**：三张数据图走几何节点，是因为它们
// 按数据节拍重画、点又密；本图恰恰相反——**低频**（计划改一次 / 一束换一次）、图元少、**文字多**
// （方位刻度 12 个 + 高度环 2 个 + 每束两段字）。文字走场景图要 QSGTextNode，为这点重画量不值。
// 同一条判据下 `ChartLabelLayer` 也是 QQuickPaintedItem——本文件与它同路。
//
// **颜色全从 QML 绑进来**：QQuickItem 没有 QPalette。随主题走的五色由 TaskPlanView.qml 绑 `palette.*`，
// 不随主题走的任务色与"够不着"的红绑 Theme 单例——主题一变绑定自己会跟，本类不留颜色事实。
//
// **字体取应用字体**：QQuickItem 没有 font 属性，而全局字体在 main 里设过（中文黑体族）。
// ============================================================
// **不能写 final**：QML 可创建的类型会被 `QQmlPrivate::QQmlElement<T>` 继承一层（它在那里
// 接管创建与所有权），final 当场把这条路封死（MSVC C3246）。
class BeamCompassItem : public QQuickPaintedItem
{
    Q_OBJECT
    QML_NAMED_ELEMENT(BeamCompass)

    // ── 计划（任务页算好，按输入顺序）──
    Q_PROPERTY(QVariantList azimuths READ azimuths WRITE setAzimuths NOTIFY planChanged)
    Q_PROPERTY(double elevation READ elevation WRITE setElevation NOTIFY planChanged)
    Q_PROPERTY(QVariantList reachable READ reachable WRITE setReachable NOTIFY planChanged)
    // order[k] = 第 k 站是第几束；空 = 计划无效（有够不着的）
    Q_PROPERTY(QVariantList order READ order WRITE setOrder NOTIFY planChanged)
    Q_PROPERTY(int pathStyle READ pathStyle WRITE setPathStyle NOTIFY planChanged)       // 仅决定路径画法，不参与采集编排
    Q_PROPERTY(bool slipRing READ slipRing WRITE setSlipRing NOTIFY planChanged) // 只影响轮末那一笔
    // ── 进度 ──
    Q_PROPERTY(int current READ current WRITE setCurrent NOTIFY currentChanged) // −1 = 没在跑
    // ── 颜色全从 QML 绑进来：随主题走的五个绑 palette.*，不随主题走的两个绑 Theme 单例 ──
    Q_PROPERTY(QColor gridColor MEMBER grid_ NOTIFY themeChanged)
    Q_PROPERTY(QColor rimColor MEMBER rim_ NOTIFY themeChanged)
    Q_PROPERTY(QColor dimColor MEMBER dim_ NOTIFY themeChanged)
    Q_PROPERTY(QColor inkColor MEMBER ink_ NOTIFY themeChanged)
    Q_PROPERTY(QColor baseColor MEMBER base_ NOTIFY themeChanged)
    Q_PROPERTY(QColor taskColor MEMBER task_ NOTIFY themeChanged) // 任务色（束、当前站）
    Q_PROPERTY(QColor badColor MEMBER bad_ NOTIFY themeChanged)   // 够不着的束

public:
    enum PathStyle { Point, Cycle, Snake };
    Q_ENUM(PathStyle)
    explicit BeamCompassItem(QQuickItem *parent = nullptr) : QQuickPaintedItem(parent)
    {
        setAntialiasing(true);
        connect(this, &BeamCompassItem::planChanged, this, [this] { update(); });
        connect(this, &BeamCompassItem::currentChanged, this, [this] { update(); });
        connect(this, &BeamCompassItem::themeChanged, this, [this] { update(); });
    }

    [[nodiscard]] QVariantList azimuths() const { return azimuths_; }
    [[nodiscard]] double elevation() const { return elevation_; }
    [[nodiscard]] QVariantList reachable() const { return reachable_; }
    [[nodiscard]] QVariantList order() const { return order_; }
    [[nodiscard]] int pathStyle() const { return pathStyle_; }
    [[nodiscard]] bool slipRing() const { return slipRing_; }
    [[nodiscard]] int current() const { return current_; }

    void setAzimuths(const QVariantList &v) { assign(azimuths_, v); }
    void setReachable(const QVariantList &v) { assign(reachable_, v); }
    void setOrder(const QVariantList &v) { assign(order_, v); }
    void setElevation(const double v) { assign(elevation_, v); }
    void setPathStyle(const int v) { assign(pathStyle_, v); }
    void setSlipRing(const bool v) { assign(slipRing_, v); }

    void setCurrent(const int v)
    {
        if (current_ == v)
            return;
        current_ = v;
        Q_EMIT currentChanged();
    }

Q_SIGNALS:
    void planChanged();
    void currentChanged();
    void themeChanged();

protected:
    void paint(QPainter *painter) override
    {
        QPainter &p = *painter;
        p.setRenderHint(QPainter::Antialiasing);
        const double R = std::min(width(), height()) / 2.0 - 36.0;
        if (R < 40.0)
            return;
        const QPointF c(width() / 2.0, height() / 2.0);
        const auto rOf = [R](const double el) { return R * (90.0 - std::clamp(el, 0.0, 90.0)) / 90.0; };
        const auto at = [c](const double az, const double r)
        {
            const double th = qDegreesToRadians(az);
            return QPointF(c.x() + r * std::sin(th), c.y() - r * std::cos(th));
        };
        const auto textAt = [&p](const QPointF &q, const double halfWidth, const QString &text)
        { p.drawText(QRectF(q.x() - halfWidth, q.y() - 9, 2 * halfWidth, 18), Qt::AlignCenter, text); };

        const QFont appFont = QGuiApplication::font();
        QFont smallFont = appFont;
        smallFont.setPointSizeF(smallFont.pointSizeF() * 0.9);
        QFont bold = appFont;
        bold.setBold(true);
        p.setFont(smallFont);

        // 高度环（外圈 = 地平 0°，圆心 = 天顶）
        p.setBrush(Qt::NoBrush);
        for (const double el : {0.0, 30.0, 60.0})
        {
            p.setPen(QPen(el == 0.0 ? rim_ : grid_, 1));
            p.drawEllipse(c, rOf(el), rOf(el));
        }
        // **刻度写在东北那根斜辐条上、再往外挪一点**：正北那根上常常落着束，两段字与束的圆圈会
        // 叠在一起（当前束的光晕更宽，叠得更凶）。45° 那根上没有方位刻度；往外偏 (10, −8) 是为了
        // 不压在束所在的那圈弧上。
        p.setPen(dim_);
        for (const double el : {30.0, 60.0})
            textAt(at(45.0, rOf(el)) + QPointF(10, -8), 16, QString::number(el, 'f', 0) + QStringLiteral("°"));

        // 方位辐条与刻度
        for (int az = 0; az < 360; az += 30)
        {
            p.setPen(QPen(grid_, 1));
            p.drawLine(c, at(az, R));
            const QString text = az == 0     ? QStringLiteral("N")
                                 : az == 90  ? QStringLiteral("E")
                                 : az == 180 ? QStringLiteral("S")
                                 : az == 270 ? QStringLiteral("W")
                                             : QString::number(az);
            p.setPen(az % 90 == 0 ? ink_ : dim_);
            textAt(at(az, R + 18), 18, text);
        }
        if (azimuths_.isEmpty())
            return;

        const bool ordered = !order_.isEmpty();
        const QColor col = ordered ? task_ : bad_;
        const double rp = rOf(elevation_);
        const qsizetype n = azimuths_.size();
        // 顺时针从 fromAz 扫 span 度（Qt 的弧：0° 在三点钟、逆时针为正，单位 1/16°）。
        const auto arc = [&](const double r, const double fromAz, const double span)
        { p.drawArc(QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r), qRound((90.0 - fromAz) * 16), qRound(-span * 16)); };
        const auto cw = [](const double a, const double b) { return std::fmod(b - a + 360.0, 360.0); };
        const auto az = [this](const qsizetype i) { return azimuths_[i].toDouble(); };
        const auto station = [this](const qsizetype k) { return static_cast<qsizetype>(order_[k].toLongLong()); };
        // 第 i 束排在第几站（−1 = 不在名单里）。**不用 QVariantList::indexOf**：那要比 QVariant
        // 的相等，而下标的整型档（int / qlonglong）取决于是谁塞进来的，比出来的结果不可靠。
        const auto stationOf = [&](const qsizetype beam)
        {
            for (qsizetype k = 0; k < order_.size(); ++k)
                if (station(k) == beam)
                    return k;
            return qsizetype(-1);
        };

        // 路径：按行进顺序顺时针连（行进顺序 = 输入顺序，人工排）
        if (ordered && n > 1)
        {
            p.setPen(QPen(col, 2));
            for (qsizetype k = 0; k + 1 < n; ++k)
                arc(rp, az(station(k)), cw(az(station(k)), az(station(k + 1))));
            const double first = az(station(0));
            const double last = az(station(n - 1));
            const double span = cw(first, last);
            const double mid = first + span / 2.0;
            if (pathStyle_ == static_cast<int>(PathStyle::Cycle))
            {
                // 轮末怎么回到首束，**取决于这台云台有没有滑环**（铭辰有、视强没有）：
                //   · 有滑环 —— 继续同向转过去，跨零即可，路径把圆合上，画成实线的一段续转；
                //   · 没有滑环 —— 跨不过 360°，只能沿原路退回，画在外侧一圈虚线。
                if (slipRing_)
                {
                    p.setPen(QPen(col, 2));
                    arc(rp, last, cw(last, first));
                    p.setPen(dim_);
                    textAt(at(last + cw(last, first) / 2.0, rp + 20), 40, QStringLiteral("轮末续转"));
                }
                else
                {
                    p.setPen(QPen(col, 1.5, Qt::DashLine));
                    arc(rp + 14, first, span);
                    p.setPen(dim_);
                    textAt(at(mid, rp + 30), 40, QStringLiteral("轮末回程"));
                }
            }
            else if (pathStyle_ == static_cast<int>(PathStyle::Snake))
            {
                p.setPen(dim_);
                textAt(at(mid, rp + 20), 30, QStringLiteral("往复"));
            }
        }

        // 束：数字 = 行进顺序；排不了时数字退回输入顺序、整体标红；够不着的那束画 ×
        for (qsizetype i = 0; i < n; ++i)
        {
            const bool reach = i < reachable_.size() && reachable_[i].toBool();
            const bool now = (i == current_); // 正在走的那一束
            const QColor bc = reach ? col : bad_;
            const QPointF q = at(az(i), rp);
            if (now) // 光晕 + 实心：**不另起一支颜色**，免得与"够不着"的红抢语义
            {
                p.setPen(QPen(bc, 1.5));
                p.setBrush(Qt::NoBrush);
                p.drawEllipse(q, 17, 17);
            }
            p.setPen(QPen(bc, 2));
            p.setBrush(now ? QBrush(bc) : QBrush(base_));
            p.drawEllipse(q, 12, 12);
            p.setFont(bold);
            p.setPen(now ? base_ : bc); // 实心底上反白
            const QString label = !reach    ? QStringLiteral("×")
                                  : ordered ? QString::number(stationOf(i) + 1)
                                            : QString::number(i + 1);
            textAt(q, 12, label);
            p.setFont(smallFont);
            p.setPen(dim_);
            textAt(at(az(i), std::max(0.0, rp - 30)), 30, QString::number(az(i), 'f', 1) + QStringLiteral("°"));
        }
    }

private:
    // 计划那几项共用一个"变了才喊"的赋值口——它们一发 planChanged、一次重画。
    template <typename T, typename U>
    void assign(T &slot, const U &value)
    {
        if (slot == value)
            return;
        slot = value;
        Q_EMIT planChanged();
    }

    QVariantList azimuths_;
    double elevation_ = 0.0;
    QVariantList reachable_;
    QVariantList order_;
    int pathStyle_ = static_cast<int>(PathStyle::Cycle);
    bool slipRing_ = false;
    int current_ = -1;

    QColor grid_, rim_, dim_, ink_, base_, task_, bad_;
};
