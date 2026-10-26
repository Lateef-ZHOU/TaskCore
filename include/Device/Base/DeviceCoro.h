#pragma once
#include <algorithm> // ranges::all_of（判齐谓词）
#include <coroutine>
#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include <QString>

#include "Device/Base/DeviceBase.h" // DeviceBase / DeviceLock + 设备类型

// ============================================================
// DeviceCoro 管理协程帧和设备锁；RunAwaiter 等待单台或同族多台设备，HeldLock 只取本帧令牌。
// TaskBase 与设备页测试协程使用同一取消链，不在这里编排具体任务。
// RunAwaiter 交回完成单据，空名单表示拒收或取消；收到空结果后，协程体必须退出。
// 看门狗及重试归 Device，等待器析构负责沿原锁撤销未完成请求。
// 完成回调排队到主线程，取消铃在主线程同步执行；两者都只弱持等待单元。
// 恢复前先摘掉续体，恢复期间局部强份额保活单元，不访问可能已销毁的等待器。
// ============================================================
// ── LockRequest：一次 run 的**锁请求**——占用者名字 + 要锁哪几台。
//    成员协程按值接收它作为第一个显式形参；promise 接管帧内副本，协程体不再使用请求内容。
//    名单收 vector 而非变参：促成 promise 构造的是协程的形参列表，那里只能是一个具体类型。
struct LockRequest
{
    QString holder;                                       // 占用者名字（只作显示：合集栏 / 设备页现问）
    std::vector<std::reference_wrapper<DeviceBase>> devs; // 要锁的名单（读作引用，底下是指针）
};

class DeviceCoro
{
public:
    struct promise_type
    {
        // ══ 准备模块 / 后处理模块（用户定）══════════════════════════════
        // 要的本质是**两件通用模块**：一个能启动协程的准备模块、一个协程结束时调用的后处理模块。
        // 顺序执行下三次顺序调用天然成立；协程下被击穿，因为**协程函数返回不代表协程结束**。
        // **落点就是本类**：构造函数 = 准备模块（试锁），析构函数 / final_suspend = 后处理模块（放锁）。
        // 于是**协程体里既没有序幕也没有收尾**，只剩业务。
        // ⚠ 靠纪律在协程体结尾放锁走不通：编排协程多是死循环，函数结尾是不可达代码，真正的出口是
        // 散在各分支里的多处 `co_return`。
        //
        // **promise 收得到协程的实参**：标准 [dcl.fct.def.coroutine] —— promise 若有能接收**协程形参
        // 列表**的构造函数，编译器就用它，否则才退回默认构造；成员协程还把**隐式对象实参**排在最前。
        // 次序（实测）：promise ctor → initial_suspend → 协程体首句。
        // 协程帧够不着函数体里的局部变量，所以东西只能从**签名**进——而签名这条路是开的。
        //
        // 只有一支构造：成员协程，隐式对象实参打头、第二个实参是锁请求（本仓库的协程都是成员函数）。
        // 没有锁请求的协程造不出 promise，编译期就挡住——"要协程参与的任务一律加锁"从约定变成了约束。
        // 编译器以帧内形参副本的左值构造 promise；非 const 引用用于移动接管名单和名称。
        // 协程签名必须按值接收 LockRequest，不能借用调用者的请求；这一点由调用处遵守。
        template <typename SelfT, typename... RestT>
        promise_type(SelfT &, LockRequest &request, RestT &&...) { engage(request); }

        // 后处理模块的**最全那一层**：帧亡即跑，连"挂起中被 destroy"也覆盖到
        // （那条路不经 final_suspend）。放锁幂等且验锁，故与 final_suspend 里那次重复无害。
        ~promise_type() { releaseAll(); }

        // 态①（两扇赋值门 delete ⇒ 允许特需析构）：promise 由编译器就地构造，从不拷贝搬家。
        promise_type(const promise_type &) = delete;
        promise_type &operator=(const promise_type &) = delete;
        promise_type &operator=(promise_type &&) = delete;

        // **本帧持有的锁**（空 = 没锁上）：协程体经 `co_await HeldLock{}` 取用，~DeviceCoro 摇铃也经此口。
        // **它就是 `lock_` 本尊**：锁在不在不另设一个 bool——令牌自己就是那个事实。
        // **交非 const 份额**：等待器要 arm 得了那支铃（换弹权止于协程帧，见 DeviceLock 类注）。
        [[nodiscard]] const std::shared_ptr<DeviceLock> &held() const noexcept { return lock_; }

    private:
        // 准备模块的身子：**造令牌 → 逐台试锁 → 成了才落槽**。
        // **没锁上就不落槽**，于是“`lock_` 非空 ⟺ 本帧持锁”是个结构事实，不靠第二本账。
        // 半途失败当场收干净（已锁上的那几台），而不是拖到后处理模块——因为那时 `lock_` 是空的、
        // 出示不出凭据。这是"锁本身即账本"的必然代价。
        void engage(LockRequest &request)
        {
            devs_ = std::move(request.devs);
            auto token = std::make_shared<DeviceLock>(std::move(request.holder));
            for (DeviceBase &d : devs_)
                if (!d.tryLock(token))
                {
                    for (DeviceBase &done : devs_)
                        done.releaseLock(token); // 收干净：没锁上的那台是空动作
                    return;                      // 不落槽 ⇒ 本帧不持锁
                }
            lock_ = std::move(token); // 全员锁上了才置位
        }

        // 后处理模块的身子：**先复位再解锁**。
        //   · 先复位：摘出即防重入，且“放过了没有”这件事也由 `lock_` 自己记着（空 = 放过了）；
        //   · 再解锁：整张名单照放。**幂等 + 验锁**是这里的承重——期间已被别人锁上的那台
        //     凭据不符，被 lockMatches 当场挡下（不动别人的锁）。
        void releaseAll() noexcept
        {
            const auto token = std::exchange(lock_, {});
            if (!token)
                return;
            for (DeviceBase &d : devs_)
                d.releaseLock(token);
        }

        std::vector<std::reference_wrapper<DeviceBase>> devs_; // 本帧请求锁哪几台（准备模块填）

        // 锁令牌持有占用者名字与取消槽（见 DeviceLock）。
        //   · 身份：指针本身即 identity——设备 lockMatches 比对的就是它；
        //   · 取消入口：调它 = 摇槽里那支铃 = 打断当前挂起的那次等待；
        //   · **换弹权止于协程帧**（见 DeviceLock 类注）；
        //   · 唯一置位点 = 准备模块（全员锁上才落槽）；唯一复位点 = 后处理模块。[仅主]
        //
        // 全员锁上后才写入 lock_："`lock_` 非空 ⟺ 本帧持锁"，不另设 bool。
        // 前提（用户定）：要协程参与的任务**一律加锁并占用设备**——promise 只能从锁请求构造，没有锁请求的协程编译不过。
        // 锁请求可以**没成**（设备被占 / 未连接）：那时 lock_ 为空，协程体第一件事就是 co_return；若竟然挂起，
        // 那一次等待没人打断得了（`await_suspend` 的 arm 处判空，不会崩）。
        //
        // **锁的寿命不绑在协程帧上**（设备槽里也持着强份额）。规矩是：**协程自己负责——它死的
        // 时候必须把自己停掉**，那一句就写在下面的 ~DeviceCoro 里。这个保证要**锁、等待器、协程帧、
        // 协程体**四者配合（协程体必须在等待器交回"没成"时 co_return）；这层耦合消不掉（用户定：
        // 给四者划清界限的做法试过，结果是踩踏），故不补。
        std::shared_ptr<DeviceLock> lock_; // 空 ⟺ 本帧没持锁

    public:

        // 仅协程机器调用（标准保证至多一次，序于 initial_suspend 之前）；铸出的是唯一
        // 拥有者（~DeviceCoro 拆帧）。body 是 complete-class 上下文，可直接按值返回。
        DeviceCoro get_return_object()
        {
            return DeviceCoro{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        std::suspend_never initial_suspend() noexcept { return {}; } // eager：调用即跑试锁

        // 一切完成路径的必经点（正常 co_return / 取消恢复后 co_return / 异常吞掉 / 试锁
        // 失败早退），且晚于体内局部量析构——**放锁点**。
        // **绕过路径（挂起中 destroy）由析构函数覆盖**：那条路不经本口，而 `~promise_type`
        // 照跑——这正是把后处理模块写在析构里（而不只写在这儿）的理由。
        std::suspend_always final_suspend() noexcept
        {
            // **后处理模块**：done 帧会滞留在持有者的槽里（TaskBase 的 coro_ 到下次开始才换），
            // 锁不能跟着滞留——故放锁不能只写在析构里。
            // releaseAll 先摘掉本帧令牌；其他持有者的份额及取消铃仍按各自寿命存在。
            releaseAll();
            return {};
        }
        void return_void() noexcept {}         // 锁的释放不在此——后处理模块全路径覆盖
        void unhandled_exception() noexcept {} // 协程体不抛；兜底吞掉（出口仍经 final_suspend）
    };

    // 句柄持有者：不可拷不可动，五件特殊成员只手写析构。构造即持活帧（h_ const、无空态）
    // ——空/换/停的槽语义外包给持有方的 unique_ptr 包装：
    //   启动 slot.reset(new DeviceCoro(runXxx()))——同类型 prvalue 直构堆对象（保证省略、
    //     不经移动；make_unique 走转发必索 move 故不可用，裸 new 是刻意的）；
    //   停止 slot.reset()；空闲 = !slot || slot->done()（判空归消费端）。
    DeviceCoro(const DeviceCoro &) = delete;
    DeviceCoro &operator=(const DeviceCoro &) = delete;

    // 全类唯一手写逻辑，两拍且顺序铁律：① 未结束先打取消入口，让挂起中的等待器体面收场
    // （取消函数置形开门、协程判取消形收尾 co_return，final_suspend 在干净栈上放锁）；
    // ② destroy 释放帧。
    // 安全取消要求协程体收到空结果后退出、不得再次挂起；ring 本身不保证到达 final_suspend。
    // 本口仍无条件 destroy，不能把协程体的退出纪律当作语言自动保证。
    // 持有者只能在帧挂起或已结束时销毁它，不得从协程正在执行的同步回调中销毁自身。
    ~DeviceCoro()
    {
        if (!h_.done())
            // 按值拷一份（不可取引用）：本次促调会把协程一路跑到 final_suspend，那里的
            // releaseAll 会把 promise 的 lock_ 清空——份额持过整个调用，才不会在自己体内被析构。
            // 空 = 已收尾（后处理模块复位过）或本帧压根没锁上——两种都无铃可摇，静默即正解。
            if (const auto lock = h_.promise().held())
                lock->ring(); // ① 优雅退出（收掉当前挂起的等待）
        h_.destroy();      // ② 释放帧
    }

    // 帧已结束（co_return 后停在 final_suspend）。"槽空闲"的另一半（判空）归包装持有者。
    [[nodiscard]] bool done() const { return h_.done(); }

    // 是否仍持有本帧锁；正常结束或试锁失败后均为假。
    [[nodiscard]] bool locked() const { return bool(h_.promise().held()); }

private:
    // 只由 promise 创建返回对象，不向外开放空句柄或重复接管同一句柄的入口。
    explicit DeviceCoro(std::coroutine_handle<promise_type> h) noexcept : h_(h) {}

    const std::coroutine_handle<promise_type> h_; // 构造即持活帧，终身不换手
};

// AwaiterBase 负责结果槽、续体和取消铃；RunAwaiter 负责设备提交与析构撤单。
// HeldLock 不挂起，不使用这套等待单元。
//
// 主线程契约：onSuspend 先提交，返回 true 后才登记续体和取消铃；提交期间不泵事件。
// 接受请求后的完成通知须排队到主线程，在 await_suspend 返回后交付；不支持 begin 内同步完成。
// 拒收返回 false，不登记续体；停止已接受请求时可以同步交回空结果。
// C++ 在调用 await_suspend 前已挂起协程。本实现延后开放恢复入口，是为避免提交尚未结束时
// 重入协程、销毁正在使用的等待器，并非因为此时协程“尚未挂起”。
//
// Resumer 只弱持 Cell。恢复前 exchange 摘掉续体，确保重入或重复触发不再次恢复。
// 回调已取得的局部强份额可能活过等待器，直至本次调用退出；弱引用失效与空续体共同保证安全。
template <typename DataT>
class AwaiterBase
{
    // Cell 只保存最终结果和续体，设备账本归 RunAwaiter。两者都由等待器长期独持，回调只弱持；
    // 已进入的回调可局部保活到返回，等待器销毁不保证弱引用立即失效，恢复仍须经过一次性续体槽。
    struct Cell
    {
        DataT data{}; // **出生态即"没成"**（DataT{} 的约定）：不挂起路径与被打断路径都如实有果
        std::coroutine_handle<DeviceCoro::promise_type> h{}; // one-shot 续体槽（全栈唯一住所）
    };

public:
    // ── Resumer：交给外界的唯一形态——**就是一支 lambda**（擦除成 std::function，不另立类型）。
    //    语义极简：**观测还在就提交数据并恢复协程**，观测没了就返回。它不问"该不该恢复"：那是
    //    调用方的事（多任务等待器自封拉铃攒账，攒齐了才打包调它一次；见 RunAwaiter）。
    //    擦除是刻意的：下游模块（设备的完成回调、令牌的取消回调）**不必认识等待
    //    器、也不必认识本类**——收一个 std::function 就够，组合的类型全程不外露。
    //    DataT 由 RunAwaiter 指定为本轮完成单据列表，await_resume 统一交出结果。
    //    **DataT 的唯一约定：默认构造 = 没成。** 出生态即它，取消铃亦是它（`resume(DataT{})`）
    //    ——空名单表示失败，故拒收和取消不必各写一份恢复代码。
    using Resumer = std::function<void(DataT)>;

    AwaiterBase(const AwaiterBase &) = delete;

    // 完成通道唯一："本就无等待"走 await_suspend 返 false 支，不设 ready 捷径。
    bool await_ready() const noexcept { return false; }

    bool await_suspend(const std::coroutine_handle<DeviceCoro::promise_type> h)
    {
        // 现造一支 Resumer：只弱观测组合——提交在前、促发在后，交换门保证一次性。
        // 提交阶段尚未登记续体；返回 false 时直接交出结果，不经 resume。
        const Resumer resume = [cell = std::weak_ptr<Cell>(cell_)](DataT data)
        {
            const auto alive = cell.lock();
            if (!alive)
                return; // Cell 已释放：迟到回调静默作废
            alive->data = std::move(data);                    // 提交在前
            if (const auto once = std::exchange(alive->h, {})) // 交换门：一次性，后到打空静默
                once.resume();                                // 促发在后；其后不碰任何成员
        };
        // 钩子只拿一样：Resumer。令牌不经钩子下发——它由派生类在**自己的构造函数**里收下、
        // 存进基类的 `lock_`，本口与派生类共用那一份。
        if (!onSuspend(resume))
            return false; // 不挂起（拒收 / 空名单）：门未开、槽已空，钩子里的同步提交只落数据

        cell_->h = h; // 置续体：派生类的钩子签名里没有 coroutine_handle，遂无从外泄
        // **挂起成立之后**才登记取消铃——而"取消"无非就是**交一个'没成'再促发**，那正是
        // Resumer 本身的语义，故铃 = `resume(DataT{})` 一句，不必再手写一份开门代码
        // （DataT{} = 没成，是本类对 DataT 的唯一约定）。若已有人交过好结果，交换门早已关上，
        // 迟到的 `resume(DataT{})` 即使仍可写入保活的 Cell，也不会再次恢复协程。
        // 收尾也不必管——**谁发出去的东西谁在自己析构里收**（见 ~RunAwaiter：协程一恢复就走出
        // co_await 全表达式，而临时量的析构必早于下一条语句，故设备停机必先于协程的下一步）。
        // 铃自足：只捕一份 Resumer（其内只弱观测组合），不碰等待器、不碰协程帧。
        // `lock_` 空 = 本帧没锁上（令牌与锁同生，见 promise_type）。按约定那种协程第一件事就是
        // co_return，到不了这里；真到了也只是这一次等待没人打断得了，而不是空指针解引用。
        if (lock_)
            lock_->arm([resume] { resume(DataT{}); });
        return true;
    }

    // 恢复前 Resumer 已摘掉续体；拒收路径从未登记续体，此处只移交结果。
    [[nodiscard]] DataT await_resume() const
    {
        return std::move(cell_->data);
    }

protected:
    // **令牌从构造函数进来**（用户定），不从 promise 里捞：协程体先 `co_await HeldLock{}` 拿到
    // 本帧的锁，再原样交给每个等待器。
    // 收益是**依赖显式**：一个等待器出示的是哪一枚锁，写在它的构造点上，不是"从所在的帧里
    // 隐式捞一枚"。连带：本类 `await_suspend` **一次都不碰 promise_type**，只用它收下的 handle
    // 去置续体——"问本帧锁上了没"那唯一一件要 promise 的事，收在 HeldLock 一处（见其类注）。
    // **代价明码**：每个 co_await 站点多一个实参，传空即那一次等待不可打断（且不报错）。
    explicit AwaiterBase(std::shared_ptr<DeviceLock> lock) noexcept : lock_(std::move(lock)) {}
    ~AwaiterBase() = default; // 从不经基类销毁（等待器恒以具体型析构）——非虚 + protected 封死裸删门

    // 本帧令牌：派生类出示给设备验身份（下单 / 撤单），本类拿它登记取消铃。
    // **protected + const**，合"protected 只放 const 数据"；const 成员顺带焊死两扇赋值门。
    const std::shared_ptr<DeviceLock> lock_;

private:
    // 挂起钩子（私有纯虚，NVI）：递交设备请求；返 false = 不挂起。
    // 只给 Resumer——令牌派生类自己就有（基类的 `lock_`），
    // promise 更不外递（那会把 final_suspend / 帧本身一并交出去，最小权限不成立）。
    virtual bool onSuspend(const Resumer &resume) = 0;

    // 等待器持唯一长期强份额，外部回调只弱持；正在执行的回调可临时保活 Cell。
    const std::shared_ptr<Cell> cell_ = std::make_shared<Cell>();
};

class HeldLock final
{
public:
    // await_suspend 读取 promise 后返回 false，直接交出令牌，不保持挂起。
    [[nodiscard]] bool await_ready() const noexcept { return false; }

    bool await_suspend(const std::coroutine_handle<DeviceCoro::promise_type> h) noexcept
    {
        lock_ = h.promise().held();
        return false; // 不挂起
    }

    [[nodiscard]] std::shared_ptr<DeviceLock> await_resume() noexcept { return std::move(lock_); }

private:
    std::shared_ptr<DeviceLock> lock_; // 出生态空 = 没锁上（与等待器的"出生态即没成"同调）
};

// 请求与结果按设备配对；DAQ 成功帧已由设备先行消费，等待器只汇总成败。
template <typename FamilyT>
struct RunOrder
{
    using ColT = typename FamilyT::CollectorBase;

    FamilyT &dev; // 借用 QML 官邸持有的后端；设备退场时先经 retiring 同步取消等待。
    std::shared_ptr<const ColT> collector; // 提交前为原型，提交时移空；只由完成回调填入本轮结果。
};


// ============================================================
// RunAwaiter<FamilyT> — 一批 run 的等待器：**下单 + 攒账 + 判齐 + 撤单**四件都在这儿。
//   if ((co_await RunAwaiter{lock, gmb, gmb.collectorFor(g, hold, movement)}).empty()) co_return; // 单台
//   const auto r = co_await RunAwaiter<DAQDeviceBase>(lock, std::move(orders));         // 多卡同采 join（名单运行期拼好）
//
// **按族模板化**：`FamilyT` 是族基（GMBDeviceBase / DAQDeviceBase），账本元素型由
// `FamilyT::CollectorBase` 给出。**批量天然族内同质**（一台云台带多张卡同时开采），族基本身就是
// 可存的类型，故单据是**数据**（设备引用 + 斗份额），"启动"就是族口 `begin` 的一次虚调用。
// 混族同批不可能——那是这笔交易付出去的东西，而它零客户。
//
// orders_ 保留设备对应关系；提交时移出原型，等待期间只保存本轮回调交付的结果。
// 输入原型即使已完成，也不能充当新请求的结果。成功结果仍复制到基类结果槽。
// 回调弱持账本，不捕获等待器 this；运行中的局部强份额可活过等待器，续体仍只能恢复一次。
// ============================================================
template <typename FamilyT>
class RunAwaiter final : public AwaiterBase<std::vector<RunOrder<FamilyT>>>
{
public:
    using ColT = typename FamilyT::CollectorBase;
    using Order = RunOrder<FamilyT>;
    using Orders = std::vector<Order>; // 单据即账本即 co_await 的产物（三位一体）
    // 基类是**依赖名**（随 FamilyT 变），故它的成员名不进本类的非受限查找——`Resumer` 要显式
    // 引进来才写得出 onSuspend 的签名。
    using Base = AwaiterBase<Orders>;
    using typename Base::Resumer;

    // 单台（最常见）：括号形，收设备引用 + 一只斗。
    RunAwaiter(std::shared_ptr<DeviceLock> lock, FamilyT &dev, std::shared_ptr<const ColT> proto)
        : Base(std::move(lock))
    {
        orders_->emplace_back(dev, std::move(proto));
    }

    // 名单形（多卡任务：卡数开始时才知道）：同族任意型号任意台数（型号已擦到
    // 族基，名单天然同质）。调用面：`co_await RunAwaiter<DAQDeviceBase>(lock, orders)`。
    RunAwaiter(std::shared_ptr<DeviceLock> lock, Orders orders)
        : AwaiterBase<Orders>(std::move(lock)), orders_(std::make_shared<Orders>(std::move(orders)))
    {
    }

    // 提交过但尚无结果的项为空，沿原锁撤单；Device 自身负责幂等、验锁和防重入。
    // 部分拒收时后续项可能仍是未提交原型，未完成项同样走既有撤单路径，不另存启动标记。
    // 停机走族口 `dev.stop(...)`（族基自己有停机口，不必经 `device()` 绕一手）。
    ~RunAwaiter()
    {
        for (const Order &o : *orders_)
            if (!o.collector || !o.collector->settled()) // 空原型被拒收也须走验锁撤单，不能解引用。
                // `this->` 是必须的：本类是模板，非依赖名查不到依赖基类 AwaiterBase 的成员。
                o.dev.stop(this->lock_); // 令牌由本器出示（构造时收下，见 AwaiterBase）
    }

    // await_resume 继承基类：全部收到本轮完成结果才返回名单，拒收或取消返回空名单。
    // Device 排队交付后，回调填入对应空项；成功时账本复制到 Cell，随后移动交给协程。
    // 这里复制的是设备引用和 Collector 份额，不复制采集数据；分层与该次复制暂时保持。

private:
    // 提交时已移空原型，排队回调只填本轮成功成品；非空即完成，不再询问 Collector 的判齐逻辑。
    // 空名单由 onSuspend 拒绝，不会进入完成回调。
    [[nodiscard]] static bool allCompleted(const Orders &orders) noexcept
    {
        return std::ranges::all_of(orders, [](const Order &o)
                                   { return bool(o.collector); });
    }

    // 逐单启动，全部接收才挂起；任一拒收则析构撤销未完成请求，已完成帧仍由设备消费。
    bool onSuspend(const Resumer &resume) override
    {
        // 令牌是构造时交进来的（住基类的 `lock_`），本口与析构都直接用，不必再抄一份。
        if (orders_->empty())
            return false; // 空名单不挂起——否则无人可促，协程永挂（本行是结构性兜底）
        for (std::size_t i = 0; i < orders_->size(); ++i)
            // begin 拒收直接返回 false；成功接收后的完成通知必须按上方契约排队交付。
            // 一单失败即恢复空结果；全部完成则交出账本，不另存完成计数。
            if (!(*orders_)[i].dev.begin(
                    std::exchange((*orders_)[i].collector, {}), // 临时份额保活到 begin 返回，账本先移空。
                    [resume, ledger = std::weak_ptr<Orders>(orders_), i](std::shared_ptr<const ColT> c)
                    {
                        const auto orders = ledger.lock();
                        if (!orders)
                            return; // 账本已释放：迟到回调静默作废
                        if (!c)
                        {
                            resume({}); // ① 空份额 = 本单没成 ⇒ 整批没成，不必等齐
                            return;
                        }
                        (*orders)[i].collector = std::move(c); // ② 只填入本轮已交付的成品
                        if (allCompleted(*orders))         // ③ 本轮结果是否到齐
                            resume(*orders); // ④ 齐了：整本提交 + 恢复（此后每单的斗都是成品）
                        // resume 可能同步销毁等待器；下方不再访问等待器或账本内容。
                    },
                    this->lock_))
                return false; // 拒收：已启动者随本器析构撤单
        return true;          // 全员收下即挂起
    }

    // 唯一长期所有者在等待器；回调弱持，调用期间以局部强份额保活账本。
    // Order 含引用成员，不赋值或改名单结构，只更新各项 Collector。
    const std::shared_ptr<Orders> orders_ = std::make_shared<Orders>();
};

// CTAD：`RunAwaiter{lock, gmb, gmb.collectorFor(t, hold, movement)}` 由第**二**实参定族（第三实参是非推导语境）。
// 名单形（`RunAwaiter<DAQDeviceBase>(lock, orders)`）须显式写族——从 vector<Order> 推不出族。
template <typename FamilyT>
RunAwaiter(std::shared_ptr<DeviceLock>, FamilyT &,
           std::shared_ptr<const typename FamilyT::CollectorBase>) -> RunAwaiter<FamilyT>;
