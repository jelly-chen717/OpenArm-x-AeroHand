#ifndef DM_DEVICE_PUB_USER_H
#define DM_DEVICE_PUB_USER_H

#include "dmcan_export.h"
#include "dmcan_types.h"
#include "dmcan_version.h"

#ifdef __cplusplus
extern "C" {
#endif


DEVICE_API void dmcan_context_create(struct dmcan_context **ctx);
DEVICE_API void dmcan_context_destroy(struct dmcan_context *ctx);
DEVICE_API void dmcan_print_version(struct dmcan_context *ctx);
DEVICE_API void dmcan_get_sdk_version(struct dmcan_context *ctx, uint32_t *version);

/*
 * 终端日志控制。日志级别：0=trace 1=debug 2=info 3=warn 4=error 5=critical 6=off。
 *
 * 默认级别 info，也可用环境变量 DMCAN_LOG_LEVEL（trace/debug/info/warn/error/critical/off）
 * 设置初始值；info/debug 写 stdout，warn 及以上写 stderr 并立即 flush。
 *
 * 注意：Release 构建默认把 trace/debug 编译裁剪掉（-DDMCAN_LOG_ACTIVE_LEVEL=0 可打开），
 * 因此即使级别设成 trace，发布件里也拿不到那两级日志。
 */
/* 设置终端日志级别：低于该级别的日志不再输出。 */
DEVICE_API void dmcan_set_log_level(int level);
/* 读取当前终端日志级别。 */
DEVICE_API int dmcan_get_log_level(void);
/* 立即刷新日志缓冲（warn 及以上本来就会立即 flush）。 */
DEVICE_API void dmcan_flush_log(void);

/*
 * 注册日志订阅，把日志交给用户自己落盘（文件/网络/自定义格式由用户决定）。
 *
 * **可以注册多个**（最多 4 个）：典型用法是"终端安静 + 文件留全 + 界面显示"各占一个订阅。
 * min_level 是**回调自己的阈值**，与终端级别互相独立：终端可用 dmcan_set_log_level(6) 静音，
 * 回调仍按自己的阈值收日志。
 *
 * 使用约定（务必遵守）：
 *   1. 回调**同步执行在产生日志的那个线程上**，不要在回调里做重活（fsync/网络/加锁）；
 *      要落盘就自己攒队列、由自己的线程写；
 *   2. 回调在 SDK 的日志锁**之外**调用，不会死锁；但回调之间可能互相穿插，按 seq 排序；
 *   3. 不要在回调里再调用 SDK 的日志接口（会递归，内部会丢弃重入日志）；
 *   4. 不要在回调里抛异常（C ABI，内部虽有兜底但不要依赖）；
 *   5. 注销后确保没有在途调用，再卸载你自己的模块。
 *
 * @param[in]  cb                日志回调；不可为 NULL
 * @param[in]  min_level         该回调自己的阈值（0=trace … 5=critical，越界 clamp）
 * @param[in]  user_data         原样回传给 `cb`
 * @param[out] out_subscription  日志订阅句柄；成功时写回
 * @return true 表示注册成功；槽位满时返回 false（`DMCAN_ERR_NO_MEMORY`）
 */
DEVICE_API bool dmcan_add_log_callback(dmcan_log_callback cb, int min_level, void *user_data,
    dmcan_log_subscription_t **out_subscription);

/**
 * @brief 注销日志订阅。
 *
 * @return true 表示注销成功；false 表示句柄非法 / 已注销
 *
 * @note 注销后该回调立即不再被调用（正在执行的那一条会先返回）。
 */
DEVICE_API bool dmcan_remove_log_callback(dmcan_log_subscription_t *subscription);

/*
 * 旧式日志回调（**已弃用**，保留仅为兼容）：等价于
 * `dmcan_add_log_callback(cb, min_level, NULL, NULL)`，但**只占一个专用旧式槽位**——
 * 重复注册 = 覆盖，`cb = NULL` 等价于 `dmcan_unregister_log_callback()`。
 * 新代码请用 `dmcan_add_log_callback()`。
 */
DEVICE_API void dmcan_set_log_callback(dmcan_log_callback cb, int min_level);
/* 注销旧式日志回调（可重复调用）；不影响 dmcan_add_log_callback 注册的订阅。 */
DEVICE_API void dmcan_unregister_log_callback(void);

/*
 * 枚举设备并刷新句柄表。
 *
 * **句柄表只增不销毁**：已经在表里的会话会复用原来的句柄（按会话对象认同一台），
 * 新出现的设备追加在表尾，已经拔掉的设备**留在表里**（`dmcan_device_is_present()`
 * 会回 false）。这样重新枚举不会让用户手里的句柄悬垂——热插拔会让重新枚举变成常态，
 * 销毁句柄等于把用户的指针变成野指针。
 *
 * @return 句柄表里的设备总数（**含已拔掉的**）；要判断某台还在不在，用
 *         `dmcan_device_is_present()`。
 */
DEVICE_API int dmcan_find_devices(struct dmcan_context *ctx);
DEVICE_API int dmcan_find_devices_with_type(struct dmcan_context *ctx, dmcan_device_type_t type);
DEVICE_API void dmcan_show_all_devices(struct dmcan_context *ctx);


/* 按句柄表下标取句柄（下标含义见 `dmcan_find_devices()`：含已拔掉的设备）。 */
DEVICE_API bool dmcan_device_get(struct dmcan_context *ctx, struct dmcan_device_handle **dev_handle, int index);

/* 该设备是否还在总线上。
   @param[out] present 设备当前是否在插；拔掉后句柄本身仍然有效，可直接查询
   @return true 表示查询成功 */
DEVICE_API bool dmcan_device_is_present(struct dmcan_device_handle *dev_handle, bool *present);

/*
 * 读取设备的枚举信息（设备类型 / PID / VID / 序列号）。
 *
 * 只读会话里缓存的枚举信息：**不需要设备处于 open 状态**，也不与设备通信，所以打开前也能调。
 * 任一 `type` / `pid` / `vid` / `serialnum` 传 NULL 表示"这一项不要"，其余项照常填充。
 *
 * @param[out] type      设备类型，取值见 dmcan_device_type_t
 * @param[out] pid       USB PID
 * @param[out] vid       USB VID
 * @param[out] serialnum 序列号；**指向库内部缓冲，不要修改也不要 free/释放**（跨模块释放是
 *                       未定义行为）；该字段只在枚举时写一次，指针随句柄生命周期有效
 */
DEVICE_API void dmcan_device_get_info(struct dmcan_device_handle *dev_handle, dmcan_device_type_t *type, int *pid,
    int *vid, unsigned char **serialnum);

DEVICE_API bool dmcan_device_open(struct dmcan_device_handle *dev_handle);
DEVICE_API void dmcan_device_close(struct dmcan_device_handle *dev_handle);

/*
 * 读设备固件信息：**一条 `CMD_GET_VERSION` 拿全** —— 发布版本号 + 构建标识。
 *
 * 为什么要合并成一个接口：这两样东西回答的问题不同，以前分散在三个接口里（其中一个名叫
 * `get_version_string`，实际返回的是 boot 的固件名、在 app 固件下恒为空），调用方要自己拼，
 * 还容易把"版本号没变"误当成"升级没生效"。
 *
 * @param[out] out 读到的信息；两个 `*_valid` 位分别表示各自那部分**这次读到了没有**。
 *                 没读到的那部分字段保持零值（版本号 0.0.0 / 空串），不会回旧值
 * @return true 表示至少读到了其中一样（版本号或构建标识）
 *
 * @note **判"升级是否生效"只看 `build_id`**：发布版本号在连续几版之间常常不变；
 *       若 `build_id_valid` 为假，那台设备就没法用软件判断升级结果。
 * @note 构建标识由固件**追加在版本应答尾部**（第 6 字节起的 ASCII，≤ 39 字节、不带结尾 NUL；
 *       见 `src/protocol/version_payload.h`）。老固件的应答只有 6 字节、没有这一段，此时
 *       `build_id_valid` 为假而 `version_valid` 仍可能为真——判"升级"时必须把这两种情况分开。
 * @note **app / boot 靠应答首字节区分**（`'A'` / `'B'`，两者都是 6 字节头 + 可选标识；
 *       老 boot 的 ASCII `bootloader …` 文本也认）。设备跑在 boot 时没有 **app 的**发布版本号，
 *       所以 `version_valid` 为假、`version` 保持 0.0.0，而 `build_id_valid` 可能为真
 *       （boot 也会带标识）—— 判"板子还停在 boot"请用 **`version_valid`**，不要用返回值。
 * @note `out->device`（型号）取自枚举表，只要设备在位就是有效的，与上面两部分读没读到无关。
 */
DEVICE_API bool dmcan_device_get_firmware_info(struct dmcan_device_handle *dev_handle, dmcan_firmware_info_t *out);

/*
 * 把固件信息格式化成人读的一行，写进 `buffer`
 * （读到版本号时形如 `USB2CANFD_D V2.0.0 [4bb41da 2026-09-24 19:25:07]`；没有构建标识时只输出
 * 型号与版本号；**只有标识、没有版本号**时输出 `型号 version not read (device may be stuck in boot) [build id]`，

 * 不会把 0.0.0 印成版本号）。
 *
 * @param[in]  info        由 `dmcan_device_get_firmware_info()` 读到的信息
 * @param[out] buffer      输出缓冲
 * @param[in]  buffer_size 缓冲大小（含结尾 '\0'）
 * @return 写进去的字符数（不含结尾 '\0'）；缓冲太小则截断，且保证以 '\0' 结尾
 *
 * @note 不访问设备（只格式化入参）；型号用 `dmcan_device_type_t` 对应的可读名字。
 *       参数非法时返回 0，`dmcan_get_last_error()` 为 `DMCAN_ERR_INVALID_ARG`。
 */
DEVICE_API uint32_t dmcan_firmware_info_to_string(const dmcan_firmware_info_t *info, char *buffer,
    uint32_t buffer_size);

DEVICE_API bool dmcan_device_enable_channel(struct dmcan_device_handle *dev_handle, uint8_t channel);
DEVICE_API bool dmcan_device_disable_channel(struct dmcan_device_handle *dev_handle, uint8_t channel);

DEVICE_API bool dmcan_device_get_channel_baudrate(struct dmcan_device_handle *dev_handle, uint8_t channel,
    dmcan_channel_can_info_t *baud_info);
DEVICE_API bool dmcan_device_get_channel_baudrate_details(struct dmcan_device_handle *dev_handle, uint8_t channel,
    dmcan_ch_baud_conf_t *config);
/*
 * 写通道波特率。
 *
 * @note **TDC（发送延迟补偿）由 SDK 自动决定开关**：CAN FD 且数据段波特率 > 1 Mbps 时自动打开，
 *       其余情况关闭。TDC 的**偏移值（SSP）由固件按数据段位定时自行计算**，上位机不参与。
 *       为什么要自动开：5 Mbps 数据相位的位时间只有 200 ns，收发器环路延迟占了大半，
 *       不补偿会采错位——真机双向压测实测到"数据相位协议错误"（错误记录 `flags.bit5`）→
 *       bus-off → 控制器发送队列被刷（固件记 `tx_failed=+32`），打开后同一组压测 `tx_failed=+0`。
 * @note `dmcan_channel_can_info_t` 里**没有** TDC 字段，所以 `get_channel_baudrate()` 读回的
 *       参数不携带该信息；要看当前生效的 TDC 状态请用 `get_channel_baudrate_details()`
 *       （`dmcan_ch_baud_conf_t::enable_tdc`）。
 * @note **经典 CAN（`canfd=false`）时数据段那四个字段也不能是 0**：固件会拒绝"经典 CAN +
 *       数据段全 0"的整条配置（回 IO 错误）。走本接口时 SDK 会自动把数据段填成与仲裁域相同的
 *       参数，调用方不用管；只有走 `..._details()` 自己填 `dmcan_ch_baud_conf_t` 时才需要显式给出。
 * @note **下发失败之后通道可能变成不可用**（实测 `enabled` 变 0、随后丢帧）：失败后请重新
 *       `dmcan_device_enable_channel()` 并读回状态确认，不要假定参数没变就没事。
 */
DEVICE_API bool dmcan_device_set_channel_baudrate(struct dmcan_device_handle *dev_handle, uint8_t channel,
    const dmcan_channel_can_info_t *baud_info);
/*
 * 按完整下发结构写通道参数（字段逐一对应固件侧布局，含 `enable_tdc` 与 `busoff_autoreset_ms`）。
 *
 * @note 结构体由调用方自己填，所以**经典 CAN 时也要给非 0 的数据段字段**（推荐填成与仲裁域
 *       相同的参数）；留 0 会被固件整条拒绝，且失败的下发可能让通道变得不可用。
 */
DEVICE_API bool dmcan_device_set_channel_baudrate_details(struct dmcan_device_handle *dev_handle, uint8_t channel,
    const dmcan_ch_baud_conf_t *config);
/*
 * 读通道的 **TDC 硬件实际生效值**（`CMD_GET_BAUD` 应答尾 3B：TDCO / TDCF / TDCV）。
 *
 * @param[out] out `valid` 为真才代表读到了生效值；为假说明"没读到"——老固件应答不足 25B，
 *                 或通道是经典 CAN / 没启用 TDC（固件这两种情况回 0，SDK 也标成无效，
 *                 免得上位机把 0 当成"生效值 0"）。`tdcv` 是硬件实测收发器延迟，
 *                 **只有成功发送过数据帧之后**才非 0。
 * @return true 表示命令成功（不看 `valid`）
 *
 * @note **下发与生效不同步**：`CMD_SETUP_BAUD` 只写 RAM 并置重载标志，硬件重载发生在固件每秒任务里，
 *       实测 0.086 ~ 1.0 s 生效。刚下发完立刻回读可能还是旧值，请等 ≥1.2 s 或轮询到期望值。
 */
DEVICE_API bool dmcan_device_get_channel_tdc(struct dmcan_device_handle *dev_handle, uint8_t channel,
    dmcan_tdc_status_t *out);

/*
 * 把**当前运行期参数**保存进设备闪存（`CMD_SAVE_PARAM`）。
 *
 * @return true 表示固件回 `ACK_SUCCESS`（**命令被受理**，实际写入由固件后台任务完成）
 *
 * @note **掉电后能保留的只有保存过的配置**：`set_channel_baudrate*()`、`set_channel_ext_config()`、
 *       `enable_channel()` 这类改动默认只影响运行期，拔插 USB / 重启就回到上次保存的那一套。
 * @note 与"恢复出厂"（管理库的 `dmadmin_recovery_factory()`）方向相反：那个是用闪存里的
 *       出厂值覆盖运行期参数，这个是反过来把运行期参数写进闪存。
 * @note 落盘有延迟（后台任务），**要验证持久化请等一会儿再断电/拔插**；返回 true 只代表受理，
 *       不代表已经写完——这一点与其它"后台命令"一样，SDK 层看不到后台 ACK 的结果。
 */
DEVICE_API bool dmcan_device_save_param(struct dmcan_device_handle *dev_handle);

/*
 * 下发硬件过滤表（`CMD_SET_FILTER`，协议 §6.1a）。
 *
 * 规则由控制器硬件执行：未通过的帧不进 RX FIFO0，**固件侧没有任何计数**，
 * 所以"对端没收到"只能靠对端自己的接收计数来判定，不要拿 `rx_dropped` 解释。
 *
 * @param[in]  channel 通道号
 * @param[in]  mode    匹配模式：0 = 白名单（未命中 → 拒收）/ 1 = 黑名单（未命中 → 正常收）
 * @param[in]  rules   规则数组；`count` 为 0 时可传 NULL（= 清空表，恢复全接受）
 * @param[in]  count   规则条数，上限 32（且标准 / 扩展各 ≤ 16）
 * @param[out] usage   固件回的用量 `{std_used, ext_used}`；可传 NULL 表示不关心
 * @return true 表示固件回了 `ACK_SUCCESS`（**受理**，不等于已生效，见下）
 *
 * @note **改表前必须先 `dmcan_device_disable_channel()`**：使能状态下发送本命令，
 *       固件回 `NACK_CALL_ERR`(4)，本接口返回 false 且表不变。
 * @note 越界参数（模式 / 字段越界、标准 ID > 0x7FF、扩展 ID > 0x1FFFFFFF、范围 `id1 > id2`、
 *       条数超限、`reserved != 0`）在 **SDK 本地**就被拒绝并置 `DMCAN_ERR_INVALID_ARG`，
 *       不会走到设备；固件的 `NACK_ARG_ERR`(3) 是第二道防线。
 * @note **下发与生效不同步**：硬件重载在固件每秒任务里做（实测 0.086 ~ 1.0 s），
 *       请等 ≥1.2 s 再发帧验证；改完要重新 `enable_channel`。
 * @note 表**不随普通下发落盘**：要断电保留必须再调 `dmcan_device_save_param()`；
 *       `dmadmin_recovery_factory()` 会把表清空（恢复全接受）。
 */
DEVICE_API bool dmcan_device_set_channel_filter(struct dmcan_device_handle *dev_handle, uint8_t channel, uint8_t mode,
    const dmcan_filter_rule_t *rules, uint16_t count, dmcan_filter_usage_t *usage);

/*
 * 回读硬件过滤表（`CMD_GET_FILTER`）：整表一次带回（最大 389B）。
 *
 * @param[out] table `rules[0..count-1]` 有效；`std_max`/`ext_max` 是固件支持的上限（当前 16/16）
 * @return true 表示固件回了 `ACK_SUCCESS` 且应答长度与 `count` 自洽
 *
 * @note 空表是合法结果（`count = 0`）：白名单模式下的空表会拒收所有帧，
 *       出厂默认是**黑名单 + 空表** = 全接受。
 */
DEVICE_API bool dmcan_device_get_channel_filter(struct dmcan_device_handle *dev_handle, uint8_t channel,
    dmcan_filter_table_t *table);

/*
 * 开关设备侧看门狗（`CMD_SWITCH_WATCH_DOG`）。
 *
 * @param[in] ms    超时（毫秒）；**只在 `state == true` 时生效**（固件要求 ≥ 50ms 才受理）
 * @param[in] state true = 使能，false = 关闭
 * @return true 表示固件回了 `ACK_SUCCESS`（"命令被接受"，不等于"设备已经不再咬"）
 *
 * @note 使能且命令被接受后，SDK 内部会起一条**喂狗线程**持续喂；`close()` 或关狗时停掉。
 * @note **关狗命令被拒（返回 false）时要当心**：本地喂狗线程仍会停掉（否则 `close()` 会卡在 join 上），
 *       而**设备侧的狗可能还在武装** —— 每过一个超时就会把所有通道 `enabled` 置 0，
 *       之后发出去的帧全部计入 `tx_dropped_ch_disabled`。出现这种情况请重新
 *       `dmcan_device_enable_channel()`，并确认固件版本支持"关狗时不校验超时下限"。
 */
DEVICE_API bool dmcan_device_enable_watchdog(struct dmcan_device_handle *dev_handle, uint16_t ms, bool state);

/*
 * 订阅设备事件（接收帧 / 发送完成 / 总线确认 / 通道错误）。
 *
 * 一个设备最多 4 个订阅者（含下面那组旧式 `hook_*` 兼容接口占用的 1 个），
 * 按注册顺序依次回调；`channel_mask` 在分发前过滤，不匹配的事件不进回调。
 *
 * @param[in]  dev_handle       设备句柄
 * @param[in]  config           订阅配置（事件位图 + 通道掩码 + 回调 + user_data）
 * @param[out] out_subscription 订阅句柄；成功时写回，失败不动
 * @return true 表示订阅成功；失败原因见 `dmcan_get_last_error()`
 *
 * @note **可以在 `dmcan_device_open()` 之前订阅**：订阅表挂在会话上，`close()` /
 *       重新 `open()`（含热插拔拔插后的重连）**都不会丢订阅**，不需要重连后重新注册。
 * @note 回调线程、事件指针有效期、以及"回调里不要 close"等约定见 `dmcan_event_callback` 的注释。
 */
DEVICE_API bool dmcan_device_subscribe(struct dmcan_device_handle *dev_handle,
    const dmcan_subscribe_config_t *config, dmcan_subscription_t **out_subscription);

/**
 * @brief 注销订阅并释放该槽位。
 *
 * @return true 表示注销成功；false 表示句柄非法 / 已注销（`DMCAN_ERR_INVALID_ARG`）
 *
 * @note `subscription` 注销后即失效，重复注销返回 false。
 * @note **可以在回调里调用**：返回后本设备不会再向该订阅投递事件。
 * @note 跨线程注销会等当前这一批事件分发结束（通常亚毫秒），不会阻塞在正在执行的回调上。
 */
DEVICE_API bool dmcan_device_unsubscribe(struct dmcan_device_handle *dev_handle,
    dmcan_subscription_t *subscription);

/* ---------------------------------------------------------------------------
 * 旧式回调（**已弃用**，保留仅为兼容 v1.1.0 的老上位机；新代码请用 dmcan_device_subscribe）
 *
 * 语义与老版本一致：同一设备同一族只有一个槽位，重复注册 = 覆盖，传 NULL = 注销。
 * 内部映射成"旧式订阅"，占用 4 个订阅槽中的 1 个（三者共用这一个槽，按事件类型分派）。
 *
 * 与老版本唯一的**行为差异**：在 `open()` 之前注册现在也生效（老版本会静默丢弃）。
 *
 *   - `hook_recv_callback`：收 `DMCAN_EVENT_RX_FRAME`，签名 `(handle, frame)`；
 *   - `hook_sent_callback`：收 `TX_COMPLETED` **和** `TX_ON_BUS`，用 `frame->ack` 区分
 *     （`ack=0` 假 echo / `ack=1` 真 echo），顺序恒为 0 → 1；
 *   - `hook_err_callback`：把 16B `dmcan_channel_error_t` 原样装回 `dmcan_rx_frame_t`
 *     （`dlen = 16`、`payload` 前 16B），与老版本逐字节一致。
 * ------------------------------------------------------------------------- */
DEVICE_API void dmcan_device_hook_recv_callback(struct dmcan_device_handle *dev_handle, dev_recv_callback callback);
DEVICE_API void dmcan_device_hook_sent_callback(struct dmcan_device_handle *dev_handle, dev_sent_callback callback);
DEVICE_API void dmcan_device_hook_err_callback(struct dmcan_device_handle *dev_handle, dev_err_callback callback);


/*
 * 单次发送 / 周期发送。
 *
 * @param[in] dlen 数据长度。**会自动向上取整到最近的 DLC**：线格式能表达的长度只有
 *                 经典 `0~8`、CAN FD 再加 `12/16/20/24/32/48/64`，所以 `dlen = 9` 会按
 *                 DLC 9 发送、**线上长度是 12 字节**（不足的部分补 0）。传进来的
 *                 `dlen` 个字节按原样发出，多出来的补 0 字节不会被写回调用方的缓冲区。
 *                 装不下的组合（经典帧 `dlen > 8`、任意帧 `dlen > 64`）返回 false 并置
 *                 `DMCAN_ERR_INVALID_ARG`。
 * @param[in] payload 数据；`dlen` 为 0 时可传 NULL
 * @return true 表示已受理（周期发送时是"任务入队"）
 */
DEVICE_API bool dmcan_device_send_can(struct dmcan_device_handle *dev_handle, uint8_t ch, uint32_t can_id, bool canfd,
    bool ext, bool rtr, bool brs, uint8_t dlen, uint8_t *payload);
DEVICE_API bool dmcan_device_send_can_details(struct dmcan_device_handle *dev_handle, uint8_t ch, uint32_t interval_ms, uint32_t send_times, uint32_t can_id,
    bool canfd, bool ext, bool rtr, bool brs,
    uint8_t dlen, uint8_t *payload);

/* 停止指定通道的周期发送任务。
   调度完全在 SDK 侧，不会向固件下发停止命令。
   @return true 表示已受理；false 表示句柄无效 */
DEVICE_API bool dmcan_device_stop_send(struct dmcan_device_handle *dev_handle, uint8_t channel);

/*
 * 设置/读取发送聚合档位（默认吞吐优先，见 `dmcan_send_profile_t` 的说明与实测数据）。
 *
 * @param[in]  profile 档位；只有枚举里列出的取值合法
 * @param[out] out     当前档位
 * @return true 表示设置/读取成功
 *
 * @note 该档位是**设备级**的（发送引擎按设备聚合，不按通道分别攒批），设置后立即生效，
 *       已入队的帧不受影响（下一笔提交开始用新窗口）。
 * @note 换档**不改变** credit/队列语义，只有"每笔 USB 装多少帧、什么时候提交"不同。
 */
DEVICE_API bool dmcan_device_set_send_profile(struct dmcan_device_handle *dev_handle, dmcan_send_profile_t profile);
DEVICE_API bool dmcan_device_get_send_profile(struct dmcan_device_handle *dev_handle, dmcan_send_profile_t *out);

/* ---------------------------------------------------------------------------
 * USB2UART 透传（模块自带的虚拟串口）
 *
 * 背景：模块除了 CANFD 的厂商接口，还有一路 CDC 虚拟串口（Windows 上是 COM 口），
 * 固件把它透传到板载 UART。SDK **不抢**那两个 CDC 接口（设备描述表里就排除了），
 * 而是以普通串口的方式打开它——所以 SDK 与 CANFD 可以同时用。
 *
 * 默认行为（可在 open 之前关掉）：**打开设备时按 VID/PID + 序列号找到属于本机的那个串口，
 * 能打开就打开**，并用默认线参数 115200 8N1 下发一次 line coding。
 * 打不开（没有这个口 / 被别的进程占用 / 波特率被固件拒绝）**不影响 CANFD 功能**，
 * 只标记 UART 不可用并打日志。
 *
 * @note **独占**：SDK 打开后该 COM 口就被本进程占住了，app 侧 py 脚本 / 串口助手会打不开；
 *       不需要 UART 时用 `dmcan_device_set_uart_auto_open(dev, false)` 关掉。
 * @note 多台同型号设备必须按序列号认（SDK 内部就是这么做的），不能按枚举顺序。
 * ------------------------------------------------------------------------- */

/*
 * 开关"打开设备时自动打开 UART"（默认开）。
 *
 * @param[in] enable false = 不自动打开；之后可以自己调 `dmcan_device_uart_open()`
 * @return true 表示设置成功
 *
 * @note 立即生效于**下一次** `dmcan_device_open()`；不影响已经打开的端口（要立刻关就用
 *       `dmcan_device_uart_close()`）。
 */
DEVICE_API bool dmcan_device_set_uart_auto_open(struct dmcan_device_handle *dev_handle, bool enable);

/*
 * 设置 UART 线参数（存下来给"自动打开"和 `dmcan_device_uart_open(NULL)` 用）。
 *
 * @param[in] config 线参数；`baud_rate = 0` 表示不改（沿用设备当前配置）
 * @return true 表示设置成功（下一次打开端口时生效）
 */
DEVICE_API bool dmcan_device_set_uart_config(struct dmcan_device_handle *dev_handle,
    const dmcan_uart_config_t *config);

/* 查询 UART 端口是否已打开。 */
DEVICE_API bool dmcan_device_uart_is_open(struct dmcan_device_handle *dev_handle, bool *is_open);

/*
 * 取当前 UART 端口名（如 `COM70` / `/dev/ttyACM0`）。
 *
 * @return true 表示端口已打开且名字已写入；false = 没打开（buffer 写空串）或参数非法
 */
DEVICE_API bool dmcan_device_uart_get_port_name(struct dmcan_device_handle *dev_handle, char *buffer,
    uint32_t buffer_size);

/*
 * 手动打开 UART 端口（按 VID/PID + 序列号找属于本机的那个）。
 *
 * @param[in] config 线参数；传 NULL 用 `dmcan_device_set_uart_config()` 存的那份（默认 115200 8N1）
 * @return true 表示已打开；false 的原因见 `dmcan_get_last_error()` 与日志（不存在/被占用/参数被拒）
 */
DEVICE_API bool dmcan_device_uart_open(struct dmcan_device_handle *dev_handle, const dmcan_uart_config_t *config);

/* 关闭 UART 端口（可重复调用）。 @note 停读线程最多等一个读超时（约 100 ms）才返回。 */
DEVICE_API void dmcan_device_uart_close(struct dmcan_device_handle *dev_handle);

/*
 * 往 UART 写一段数据（同步；写不完算失败）。
 *
 * @return true 表示全部写完
 * @note 收数据走订阅：`DMCAN_EVENT_UART_RX`（负载 `dmcan_uart_chunk_t`）。
 */
DEVICE_API bool dmcan_device_uart_write(struct dmcan_device_handle *dev_handle, const uint8_t *data, uint32_t length);

/* 读 UART 统计（收/发字节、丢字节、错误次数、是否打开）。 */
DEVICE_API bool dmcan_device_get_uart_stats(struct dmcan_device_handle *dev_handle, dmcan_uart_stats_t *out);

/* 读写通道扩展配置（重传策略与 bus-off 自动恢复间隔）。
   实现为"读回当前配置 → 只改这两个字段 → 写回"，因此不会影响该通道的
   波特率、采样点、CAN FD 开关等其他参数。
   @return true 表示成功 */
DEVICE_API bool dmcan_device_get_channel_ext_config(struct dmcan_device_handle *dev_handle, uint8_t channel,
    dmcan_channel_ext_config_t *config);
DEVICE_API bool dmcan_device_set_channel_ext_config(struct dmcan_device_handle *dev_handle, uint8_t channel,
    const dmcan_channel_ext_config_t *config);

/*
 * 纯函数工具：DLC ↔ 字节长度换算（不碰设备、不需要句柄）。
 *
 * 线格式里载荷长度只能用 DLC 表达，合法取值只有：
 *   经典 CAN `0~8`；CAN FD 再加 `12/16/20/24/32/48/64`（DLC 9~15）。
 *
 * @return 长度 → DLC **向上取整**到最小能装下它的那一档（`9~12 → 9`、`13~16 → 10`…）；
 *         DLC → 长度按表原样返回（`9 → 12`、`15 → 64`）。
 *         **输入越界时回 -1** 并置 `DMCAN_ERR_INVALID_ARG`：`dlen < 0` / `dlen > 64` /
 *         `dlc < 0` / `dlc > 15`。
 *
 * @note 早期实现让 `9/10/11` 这类长度落到 `switch` 的 `default` 后**静默回 0**，于是发送
 *       接口返回成功、对端却收到一个 0 字节帧（数据静默丢失，真机踩过）；DLC 超范围时还会
 *       按公式外推出 80/1424 这种长度。前者改成向上取整，后者改成回 -1。
 */
DEVICE_API int dmcan_utils_get_dlc_from_len(int dlen);
DEVICE_API int dmcan_utils_get_len_from_dlc(int dlc);

/* 读取设备的模块能力位（位定义见 dmcan_types.h 的 DMCAN_MODULE_FEATURE_*）。
   数值在 dmcan_device_open() 打开设备时读一次 CMD_GET_MODULE_INFO 并缓存，本函数不重新查询设备。
   能力位只表示"这台设备能做什么"，不表示任何当前状态（例如终端电阻当前是通还是断）。
   @param[out] features 能力位图；读不到时写 0
   @return true 表示打开设备时确实读到了能力位；false 表示设备没给出应答，
           此时 *features 为 0，调用方应按"无任何能力"处理 */
DEVICE_API bool dmcan_device_get_module_features(struct dmcan_device_handle *dev_handle, uint16_t *features);

/*
 * 读取本线程上一次 API 调用的错误码。
 *
 * - 返回 bool 的接口失败时，用它区分"参数非法 / 设备没开 / 与设备交互失败"等原因；
 * - 错误码是**线程局部**的：多线程各自独立，互不覆盖；
 * - 每次调用 API 都会重新置位（成功即 `DMCAN_OK`），所以失败后要立刻读；
 * - 读取本身不清除，要清用 `dmcan_clear_last_error()`。
 */
DEVICE_API dmcan_status_t dmcan_get_last_error(void);

/* 手动把本线程的错误码清成 DMCAN_OK（一般不需要：每次 API 调用都会重置）。 */
DEVICE_API void dmcan_clear_last_error(void);

/*
 * 开关通道的**软件终端电阻**（`CMD_SWITCH_CAN_RES`）。
 *
 * @return true 表示**实际状态已经等于请求状态**（不是"命令被受理"）——见下面第 3 条
 *
 * @note 三条硬约束（协议 §6 第 25 项）：
 *       1. **状态不写 flash**：设备重启 / 重新枚举后电阻会回到默认，**必须重新下发**；
 *          上位机在"拔插之后"的恢复流程里要把它算上（见 README 的热插拔一节）。
 *       2. **写要求通道已使能**：通道没使能时这次写会被固件**静默忽略**（应答状态字节回 0）。
 *          本接口按状态字节判定，因此这种情况返回 **false**（并打一条告警），不会谎报成功。
 *       3. **能力位门控**：`dmcan_device_get_module_features()` 的 bit0
 *          （`DMCAN_MODULE_FEATURE_CAN_RES_SW`）为 0 时说明这台设备没有软件控电阻能力
 *          （老硬件由板上物理开关决定），本接口直接拒绝下发。
 */
DEVICE_API bool dmcan_device_enable_can_res(struct dmcan_device_handle *dev_handle, uint8_t channel, bool state);

/* 读软件终端电阻的当前状态（1B 读形式）。能力位为 0 的老硬件读回恒 0。 */
DEVICE_API bool dmcan_device_get_can_res(struct dmcan_device_handle *dev_handle, uint8_t channel, bool *enabled);

/* 开关通道的总线 echo（真 echo，下发 CMD_SET_ECHO）。
   打开后固件把**真正发上总线**的帧按 dir=SEND、ack=1 回传，与接收帧共用 0x81 流；
   SDK 会为这些帧分配非零 echo_id（0 表示该帧不请求 echo，固件不回传）。
   @return true 表示固件确认成功；此时该通道的 sent 回调会开始收到 ack=1 的事件
           （ack=0 仍是 SDK 本地"已交给设备"的假 echo 事件）。 */
DEVICE_API bool dmcan_device_set_channel_echo(struct dmcan_device_handle *dev_handle, uint8_t channel, bool enable);

/*
 * 取 echo 诊断计数（只给计数，不产生逐帧事件）。
 *
 * timeout_count  = 超过阈值仍未收到总线确认的帧数（累计）；
 * late_discarded = 超时后才回来、按迟到丢弃的 echo 条数（累计）；
 * pending        = 当前仍在等总线确认的帧数；
 * timeout_ms     = 当前超时阈值。
 */
DEVICE_API bool dmcan_device_get_echo_stats(struct dmcan_device_handle *dev_handle, dmcan_echo_stats_t *stats);

/*
 * 主动清一次通道的 bus-off（下发 `CMD_CLEAR_BUS_OFF`，1B 通道号）。
 *
 * @return true 表示固件回 `ACK_SUCCESS`。**不在 bus-off 状态时同样回 `ACK_SUCCESS`（幂等）**，
 *         所以这个返回值只说明命令被接受，不代表"确实从 bus-off 恢复了"——
 *         要判定恢复，请接着调 `dmcan_device_get_bus_state()` 看 `bus_state`。
 */
DEVICE_API bool dmcan_device_clear_bus_off(struct dmcan_device_handle *dev_handle, uint8_t channel);

/*
 * 读最近一次**周期上报**的通道统计快照（固件每秒推一条，走状态流，不是命令应答）。
 *
 * @param[in]  channel 通道号
 * @param[out] out     最近一次快照；没收到过时清零
 * @return true 表示收到过至少一条该通道的上报；
 *         false 表示"还没收到过"或参数/状态非法（原因见 `dmcan_get_last_error()`）
 *
 * @note 两条使用约定：
 *       1. 值最多旧 1 秒，别当实时值用；
 *       2. 判"现在有没有丢包"要**连续读两次看增量**，单次绝对值可能是很久以前的累计。
 *          `false` 与"读到全 0"必须分开处理：前者是还不知道，后者是真的没丢过。
 *
 * @note 快照里 `drop_flags` 之后的字段来自**另外两条周期记录**（`0x05` 接收/echo 侧细分、
 *       `0x07` 发送侧细分），用来把 `rx_dropped` / `tx_dropped` 这两个合计拆到具体原因
 *       （"设备内部写不进缓冲" vs "上位机没把数据读走"）。两条记录各自独立到达，
 *       所以**先看 `drop_flags` 再信对应字段**——位为 0 时该组字段恒 0，不代表没丢过。
 */
DEVICE_API bool dmcan_device_get_channel_stats(struct dmcan_device_handle *dev_handle, uint8_t channel,
    dmcan_channel_stats_t *out);

/*
 * 按需读一次指定通道的总线状态（下发 `CMD_GET_BUS_STATE`，1B 通道号 → 5B 应答）。
 *
 * 与错误记录回调的区别：错误记录只在**设备侧真的产生错误**时才来，拿不到"现在是什么状态"；
 * 本接口是主动查询，任何时候都能读，适合做"等待恢复""确认已退出 bus-off"这类判定。
 *
 * @param[in]  channel 通道号
 * @param[out] out     状态快照；失败时清零，不会留上一次的值
 * @return true 表示固件回了 `ACK_SUCCESS` 且应答长度够解析（失败原因见 `dmcan_get_last_error()`）
 */
DEVICE_API bool dmcan_device_get_bus_state(struct dmcan_device_handle *dev_handle, uint8_t channel,
    dmcan_bus_state_t *out);

/* 设置 echo 超时阈值（毫秒，内部 clamp 到 [10, 60000]）。 */
DEVICE_API void dmcan_device_set_echo_timeout_ms(struct dmcan_device_handle *dev_handle, uint32_t timeout_ms);

/*
 * 注册热插拔回调（同一个 context 只保留一个，重复注册以最后一次为准）。
 *
 * @param[in] cb        事件回调；传 NULL 等价于注销
 * @param[in] user_data 原样回传给 cb，SDK 不解释也不释放
 * @return true 表示监听已起（设备变化会被报上来）；false 只表示参数非法或监听起不来。
 *
 * @note **本机 libusb 的原生热插拔能力位常常是 0（本仓库实测如此）**，但注册照样会成功：
 *       SDK 内部用"低频枚举比对"兜底，两条源喂同一个回调。因此不要用
 *       `dmcan_context_hotplug_supported()` 来决定要不要注册。
 *
 * @note 事件**只报描述表里认识的设备**（按 VID/PID 判定），无关 USB 设备不会打扰你。
 * @note 线程与调用约束见 dmcan_types.h 里 `dmcan_hotplug_callback` 的注释。
 */
DEVICE_API bool dmcan_context_register_hotplug(struct dmcan_context *ctx, dmcan_hotplug_callback cb, void *user_data);

/* 注销热插拔回调并停掉监听（可重复调用）。
   @note **不要在热插拔回调里调用**（会死锁，见回调注释）。 */
DEVICE_API void dmcan_context_unregister_hotplug(struct dmcan_context *ctx);

/* 本机 libusb 的**原生**热插拔能力是否可用（诊断用）。
   @note 返回 false 不代表这个接口没用：SDK 用低频轮询兜底，注册照样成功。 */
DEVICE_API bool dmcan_context_hotplug_supported(struct dmcan_context *ctx);

/*
 * 校验"这次编译用的头文件"与"手里的库"是否配套。
 *
 * @param[in] expected_version 通常直接传头文件里的 `DMCAN_VERSION_INT`
 * @return true 表示库编译时的版本就是这个值（配套）；false 表示不配套——**别继续用**，
 *         ABI 变更后会直接表现为结构体错位
 *
 * @note 库里记的版本是发布时写死的，用户改自己的头文件副本只会让这里返回 false，
 *       改变不了库的行为。建议在 `dmcan_context_create()` 之后调一次。
 */
DEVICE_API bool dmcan_check_version(uint32_t expected_version);

/*
 * 批量/递增发送（v1.1.0 的 `send_can_details` 里那套 ID 递增/数据递增的等价物）。
 *
 * 配置结构体与完整语义（ID 递增区间、数据进位规则、停止条件、边界处理）见
 * dmcan_types.h 的 `dmcan_bulk_send_config_t`。这些语义由离线用例钉住
 * （tests/test_bulk_send_plan.cpp），不是口头约定。
 *
 * @return true 表示已受理（周期任务入队）；false 表示参数非法或设备未打开
 *
 * @note 与 v1.1.0 的差别：那边是 14 个标量参数、同名同形；这边是配置结构体 + 新函数名。
 *       2.0.0 整体对外不兼容（v1.1.0 的 get_version/通道配置结构体等都已改变）。
 */
DEVICE_API bool dmcan_device_send_can_bulk(struct dmcan_device_handle *dev_handle, uint8_t ch,
    const dmcan_bulk_send_config_t *config);

/*
 * 一次交一批帧（v1.1.0 的 `fill_can_queue` + `can_queue_send` 两段式的数组版）。
 *
 * 与两段式相比：没有跨调用残留状态（不存在"填了忘发"），一次调用即可提交。
 * 帧会逐条进发送队列，由发送引擎按现有聚合策略合并成少量 USB 提交——v1.1.0 手工攒
 * 字节流想省下的 USB 笔数，现在由引擎自动完成。
 *
 * @param[in] frames 帧数组；count 为 0 或指针为空都算参数非法
 * @param[in] count  帧数
 * @return true 表示这批全部提交；false 表示参数非法或提交过程中被拒（已提交的不撤回）
 *
 * @note 参数校验是**全部先做**的：某一帧参数非法时不会发出前面那些帧
 *       （2026-09-20 真机实测抓到过反例——预校验只挡了 `dlen > 64`，经典帧 `dlen = 64` 漏过去，
 *       第 8 帧被入队拒绝时前 7 帧已经上了总线；现在预校验与入队共用同一条规则）。
 * @note 每帧的 `dlen` 都会**自动向上取整到最近的 DLC**（不足的部分补 0，口径同
 *       `dmcan_device_send_can()`）；只有"装不下"的组合（经典帧 `dlen > 8`、`dlen > 64`）
 *       才整批拒收并置 `DMCAN_ERR_INVALID_ARG`。
 */
DEVICE_API bool dmcan_device_send_can_batch(struct dmcan_device_handle *dev_handle, uint8_t ch,
    const dmcan_frame_t *frames, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif // DM_DEVICE_PUB_USER_H
