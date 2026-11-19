#ifndef DMCAN_TYPES_H
#define DMCAN_TYPES_H

#include <stdint.h>
#include <stdbool.h>
/* 这里原来 include 了 <cstddef>（C++ 头），会直接挡住 C 消费方编译
   （examples/minimal_c_api.c 用 C 编译时实测 “cstddef: No such file or directory”）；
   本头文件并未使用 size_t，因此直接移除，保证对外头文件 C/C++ 都能用。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 前置声明 + typedef：C 里 struct tag 不能当类型名用（写 `dmcan_context *` 会报
   "unknown type name"），而本 SDK 对外承诺是 C API。这里补 typedef 后 C/C++ 都能用，
   只是加了个类型别名，不改任何结构体布局（对 C++ 与 ABI 无影响）。 */
typedef struct dmcan_context dmcan_context;
typedef struct dmcan_device_handle dmcan_device_handle;

/* 模块能力位（`CMD_GET_MODULE_INFO` 应答里 `module_features` 的位定义）。
   语义是"这台设备支持什么"的静态能力，不是当前状态；老固件/老硬件一律为 0。 */
#define DMCAN_MODULE_FEATURE_CAN_RES_SW (1u << 0) /* bit0 = 支持 CMD_SWITCH_CAN_RES 软件开关终端电阻 */

#pragma pack(push, 1)

typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t build;
} version_t;

typedef enum
{
    USB2CANFD_S = 0,
    USB2CANFD_D,
    LINKX4C,
    UNKNOWN
} dmcan_device_type_t;


/**
 * @brief 设备固件信息：一次拿全"发布版本"与"构建标识"。
 *
 * 两个字段回答两个不同问题，**不要混用**：
 * - `version`（配 `version_valid`）：**发布版本号**，给人看、用来判断"我的上位机支不支持这台设备"；
 * - `build_id`（配 `build_id_valid`）：**构建标识**（短 hash / 构建号 + 时间），
 *   机器可比，用来判断"升级是不是真的生效"、以及给测试报告留溯源。
 *
 * @note 判"升级是否生效"**只看 `build_id`** —— 发布版本号在连续几版之间常常不变；
 *       若固件没提供构建标识（`build_id_valid` 为假），那台设备就**没法**用软件判断升级结果。
 */
typedef struct
{
    dmcan_device_type_t device; /**< 设备型号（单路 / 双路 / 四路） */
    version_t version;          /**< 发布版本号（major/minor/build） */
    bool version_valid;         /**< 版本号是否读到；false（含设备跑在 boot）时 `version` 恒为 0.0.0 */
    char build_id[40];          /**< 构建标识字符串；读不到时为空串 */
    bool build_id_valid;        /**< 构建标识是否读到 */
} dmcan_firmware_info_t;

typedef struct
{
    bool canfd;
    bool non_iso;
    bool retrans;
    uint32_t can_baudrate;
    /* 数据段波特率。**> 1 Mbps 时 SDK 会自动打开 TDC**（发送延迟补偿），
       偏移值由固件自己算；本结构不含 TDC 开关，要读当前状态用
       `dmcan_device_get_channel_baudrate_details()`。 */
    uint32_t canfd_baudrate;
    float can_sp;
    float canfd_sp;
} dmcan_channel_can_info_t;


/**
 * @brief 通道的 MCAN 节点模式（`dmcan_ch_baud_conf_t::mode` 的取值）。
 *
 * 与固件/HPM 驱动的 `mcan_node_mode_t` **同口径**（值必须一致，SDK 只透传）：
 * 内部回环是"不外发、芯片内回到接收路径"，外部回环是"帧照常驱动总线、同时回环到自己的
 * 接收路径"——后者**不依赖外部 ACK**，所以单个模块（不接对端）就能自发自收，
 * 适合自测与性能基线；也正因为帧真的上了总线，别和正常对端混用。
 *
 * @note 通过 `dmcan_device_set_channel_baudrate_details()` 下发（配置的一名字段）；
 *       固件在"每秒重载任务"里重新初始化 MCAN 才生效，实测下发到生效 0.086 ~ 1.0 s。
 */
typedef enum
{
    DMCAN_CHANNEL_MODE_NORMAL = 0,            /**< 正常收发（默认） */
    DMCAN_CHANNEL_MODE_LOOPBACK_INTERNAL = 1, /**< 内部回环：TX 不外发，帧在芯片内回到接收路径 */
    DMCAN_CHANNEL_MODE_LOOPBACK_EXTERNAL = 2, /**< 外部回环：帧照常驱动总线并回环到自己；不依赖外部 ACK */
    DMCAN_CHANNEL_MODE_LISTEN_ONLY = 3        /**< 只听不发（不驱动总线、不回 ACK） */
} dmcan_channel_mode_t;

typedef struct
{
    uint8_t enable_canfd  : 1;
    uint8_t enable_tdc    : 1;
    uint8_t non_iso       : 1;
    uint8_t enable_retrans: 1;
    uint8_t mode          : 4; /**< 节点模式，取值见 `dmcan_channel_mode_t`（默认 0 = 正常收发） */

    uint16_t nominal_brp;
    uint16_t nominal_seg1;
    uint16_t nominal_seg2;
    uint16_t nominal_sjw;

    uint16_t data_brp;
    uint16_t data_seg1;
    uint16_t data_seg2;
    uint16_t data_sjw;

    /* bus-off 后自动恢复的间隔，单位毫秒；0 = 不自动恢复，由上位机重新初始化 */
    uint16_t busoff_autoreset_ms;

    /* ---- 配置版本 4 起追加的 TDC（发送延迟补偿）字段，共 3B ----
       `CMD_SETUP_BAUD` 载荷由 20B（1+19）变成 23B（1+22）；固件仍兼容旧的 20B
       （缺的 3B 按 0 = 自动处理）。`CMD_GET_BAUD` 应答前 22B 是**下发原值逐字节回显**，
       之后还有 3B 是硬件实际生效值，见 `dmcan_tdc_status_t`。 */
    uint8_t tdc_offset; /**< 二次采样点偏移，单位数据段 mtq；**0 = 自动**（取 DTSEG1+2），1~127 = 指定值 */
    uint8_t tdc_filter; /**< TDC 滤波窗口，单位数据段 mtq；**0 = 跟随 `tdc_offset`**，1~127 = 指定值 */
    uint8_t reserved;   /**< 预留，恒 0（固件逐字节回显，便于以后再加字段） */

} dmcan_ch_baud_conf_t;

/*
 * 通道的 TDC **硬件实际生效值**（`dmcan_device_get_channel_tdc()` 读回）。
 *
 * 与 `dmcan_ch_baud_conf_t` 里的 `tdc_offset`/`tdc_filter` 分开命名是刻意的：
 * 那两个是"上位机希望下发的值"（会被固件原样回显），这里是"硬件真正用上的值"——
 * `tdc_offset=0`（自动）时固件会取 `DTSEG1+2`，两个值就不一样。
 *
 * 生效时机：`CMD_SETUP_BAUD` 只写 RAM 并置重载标志，硬件重载发生在固件每秒任务里，
 * 实测下发到生效 **0.086 ~ 1.0 s**；下发后立刻回读可能还是旧值，**这不是失败**。
 */
typedef struct
{
    bool valid;   /**< 是否读到硬件实际值；老固件（应答 <25B）或 TDC 未启用时为 false */
    uint8_t tdco; /**< 硬件生效的二次采样点偏移 */
    uint8_t tdcf; /**< 硬件生效的滤波窗口 */
    uint8_t tdcv; /**< 硬件实测收发器延迟；**只有成功发送过数据帧后**才非 0，从未发过恒 0 属正常 */
} dmcan_tdc_status_t;

/* 通道扩展配置。
   刻意与 dmcan_channel_can_info_t / dmcan_ch_baud_conf_t 分开，避免再改既有结构体的 ABI。 */
typedef struct
{
    uint8_t enable_retrans;       /* 1 = 自动重传（默认）；0 = 一次性发送 */
    uint16_t busoff_autoreset_ms; /* bus-off 后自动恢复间隔，单位毫秒；0 = 不自动恢复 */
} dmcan_channel_ext_config_t;

/*
 * 硬件过滤表（`CMD_SET_FILTER` / `CMD_GET_FILTER`，协议 §6.1a）。
 *
 * 规则由**控制器硬件**执行：未通过的帧根本不进 RX FIFO0，固件看不到，所以
 * **被过滤掉的帧没有任何计数**——上位机不能拿 `rx_dropped` 之类的计数解释"没收到"。
 *
 * 上限 16 条标准 ID + 16 条扩展 ID，总数 ≤ 32；只支持存 RX FIFO0。改表前必须先
 * `dmcan_device_disable_channel()`，改完再 `dmcan_device_enable_channel()`。
 */

/** @brief 规则的 ID 类型（`dmcan_filter_rule_t::id_type` 取值） */
typedef enum
{
    DMCAN_FILTER_ID_STD = 0, /**< 标准帧（11 位 ID） */
    DMCAN_FILTER_ID_EXT = 1  /**< 扩展帧（29 位 ID） */
} dmcan_filter_id_type_t;

/** @brief 规则的匹配类型（`dmcan_filter_rule_t::type` 取值） */
typedef enum
{
    DMCAN_FILTER_TYPE_RANGE = 0,   /**< 范围：`id1 ≤ rx_id ≤ id2`（要求 `id1 ≤ id2`） */
    DMCAN_FILTER_TYPE_DUAL_ID = 1, /**< 双 ID 精确：`rx_id == id1` 或 `rx_id == id2` */
    DMCAN_FILTER_TYPE_MASK = 2     /**< 掩码：`(rx_id & id2) == (id1 & id2)` */
} dmcan_filter_rule_type_t;

/** @brief 规则的命中动作（`dmcan_filter_rule_t::action` 取值） */
typedef enum
{
    DMCAN_FILTER_ACTION_FIFO0 = 0,    /**< 存 RX FIFO0（正常接收） */
    DMCAN_FILTER_ACTION_REJECT = 1,   /**< 拒收 */
    DMCAN_FILTER_ACTION_PRIORITY = 2  /**< 置优先级并存 RX FIFO0 */
} dmcan_filter_action_t;

/** @brief 表的匹配模式（`dmcan_filter_table_t::mode` 取值） */
typedef enum
{
    DMCAN_FILTER_MODE_WHITELIST = 0, /**< 白名单：未命中任何规则 → 拒收 */
    DMCAN_FILTER_MODE_BLACKLIST = 1  /**< 黑名单：未命中 → 正常接收 */
} dmcan_filter_mode_t;

/** @brief 单条过滤规则（12B，与固件侧布局逐字节一致；u32 一律小端） */
typedef struct
{
    uint8_t id_type;  /**< 取值见 `dmcan_filter_id_type_t` */
    uint8_t type;     /**< 取值见 `dmcan_filter_rule_type_t` */
    uint8_t action;   /**< 取值见 `dmcan_filter_action_t` */
    uint8_t reserved; /**< 预留，**必须填 0**（非 0 固件回 `NACK_ARG_ERR`） */
    uint32_t id1;     /**< 含义随 `type` 变化；标准帧 ≤ `0x7FF`，扩展帧 ≤ `0x1FFFFFFF` */
    uint32_t id2;     /**< 含义随 `type` 变化；取值范围同 `id1` */
} dmcan_filter_rule_t;

/** @brief 单张表的规则条数上限（标准 / 扩展各 16，总数 ≤ 32） */
#define DMCAN_FILTER_RULE_MAX 32
#define DMCAN_FILTER_STD_RULE_MAX 16
#define DMCAN_FILTER_EXT_RULE_MAX 16

/*
 * `CMD_SET_FILTER` 应答：固件回的两条用量（各 1B）。
 * 这是**受理结果**，不是"已经生效"——硬件重载在固件每秒任务里做（≤1 s）。
 */
typedef struct
{
    uint8_t std_used; /**< 表里标准 ID 规则的条数 */
    uint8_t ext_used; /**< 表里扩展 ID 规则的条数 */
} dmcan_filter_usage_t;

/*
 * `CMD_GET_FILTER` 应答解析结果：与应答数据区**逐字节对应**（整表一次带回，最大 389B）。
 *
 * `std_max` / `ext_max` 是固件支持的上限（当前 16/16），`count` 是表里实际条数；
 * 只有 `rules[0..count-1]` 有效，其余为 0。
 */
typedef struct
{
    uint8_t mode;     /**< 当前模式，取值见 `dmcan_filter_mode_t` */
    uint8_t std_max;  /**< 固件支持的标准 ID 规则上限 */
    uint8_t ext_max;  /**< 固件支持的扩展 ID 规则上限 */
    uint16_t count;   /**< 表里实际规则条数 */
    dmcan_filter_rule_t rules[DMCAN_FILTER_RULE_MAX]; /**< 规则数组，按匹配顺序排列 */
} dmcan_filter_table_t;

typedef struct
{
    uint32_t can_id;
    /* 单位 µs。真 echo / 接收帧 / 错误记录 = 设备时间戳（设备自己的计数器）；
       假 echo = 上位机系统时间（UTC epoch）。两者时基不同，只能各自排序，不能相减。 */
    uint64_t timestamp;
    uint8_t channel;
    uint8_t ext;
    uint8_t rtr;
    uint8_t canfd;
    uint8_t brs;
    uint8_t esi;
    uint8_t ack;
    uint8_t dlen;
    uint8_t payload[64];
} dmcan_rx_frame_t;

/*
 * 通道错误记录（`DMCAN_EVENT_CHANNEL_ERROR` 事件负载）。
 *
 * 16B，与固件 `dmcan_channel_error_payload_t` 逐字节一致；语义见
 * `tests/CONTROL_PROTOCOL.md` §7.1（`flags` 是"最近一次错误中断的原因位"，
 * `bus_state` 才是"上报快照生成时的恢复结果"，两者不是同一时点）。
 */
typedef struct
{
    uint64_t timestamp_us; /**< 设备时间戳，单位 µs */
    uint16_t error_code;   /**< 错误位图 */
    uint8_t bus_state;     /**< 0 = active / 1 = warning / 2 = passive / 3 = bus-off */
    uint8_t lec;           /**< 最近一次错误码（总线相位） */
    uint8_t dlec;          /**< 数据相位错误码 */
    uint8_t tec;           /**< 发送错误计数 */
    uint8_t rec;           /**< 接收错误计数 */
    uint8_t flags;         /**< 错误原因位（见协议 §7.1） */
} dmcan_channel_error_t;

typedef struct
{
    uint32_t can_id;
    uint8_t channel;
    uint8_t ext;
    uint8_t rtr;
    uint8_t canfd;
    uint8_t brs;
    uint8_t esi;
    uint8_t dlen;
    uint8_t payload[64];

} dmcan_tx_frame_t;

/*
 * echo 诊断计数（只给计数，不产生逐帧事件）。
 *
 * `ack=1` 只说明"收到了总线确认"；收不到时用户无法区分"还没回来"和"丢了"，
 * 这里把"超过阈值仍未收到确认"的帧数单独计数。要指认具体是哪一帧需要 echo_id，
 * 当前对外结构体不暴露它，所以只保留计数口径。
 * 计数在"设备打开"期间累计，重新打开设备后从 0 开始。
 */
typedef struct
{
    uint64_t timeout_count;  /* 超过阈值仍未收到总线确认的帧数，累计 */
    uint64_t late_discarded; /* 超时后才回来、按迟到丢弃的 echo 条数，累计 */
    uint32_t pending;        /* 当前仍在等总线确认的帧数 */
    uint32_t timeout_ms;     /* 当前超时阈值，单位毫秒 */
} dmcan_echo_stats_t;


#pragma pack(pop)


typedef void (*dev_recv_callback)(dmcan_device_handle *handle, dmcan_rx_frame_t *rec_frame);
typedef void (*dev_sent_callback)(dmcan_device_handle *handle, dmcan_rx_frame_t *sent_frame);
typedef void (*dev_err_callback)(dmcan_device_handle *handle, dmcan_rx_frame_t *err_frame);

/*
 * 事件类型位图（可按位或组合成订阅配置里的 `events`）。
 *
 * 发送侧刻意拆成两位：`TX_COMPLETED` 只证明"帧已交给设备"（USB 提交完成，假 echo），
 * `TX_ON_BUS` 才证明"帧已在总线上被确认"（固件回传，真 echo，带硬件时间戳）。
 * 同一帧恒为 `TX_COMPLETED` → `TX_ON_BUS`（顺序由 SDK 保证）。
 */
typedef enum
{
    DMCAN_EVENT_RX_FRAME      = 1u << 0, /**< 总线收到的帧 */
    DMCAN_EVENT_TX_COMPLETED  = 1u << 1, /**< 假 echo：这笔 USB 提交已完成 */
    DMCAN_EVENT_TX_ON_BUS     = 1u << 2, /**< 真 echo：固件回报该帧已在总线上被确认 */
    DMCAN_EVENT_CHANNEL_ERROR = 1u << 3, /**< 通道错误记录 */
    DMCAN_EVENT_UART_RX       = 1u << 4  /**< USB2UART 收到数据（事件负载见 `dmcan_uart_chunk_t`；
                                              UART 没有通道概念，`channel` 恒为 0） */
} dmcan_event_flag_t;

/** 通道掩码：bit N = 通道 N。"全部通道"必须用这个宏；**0 表示什么都没选**（参数错） */
#define DMCAN_CHANNEL_MASK_ALL (0xFFFFFFFFu)

/**
 * @brief UART（USB2UART 虚拟串口）收数据的事件负载。
 *
 * @note 一次回调 = 操作系统这次给到的一段连续字节，**边界不是协议帧**；
 *       `data` 只在本次回调期间有效，要留存自己拷贝。
 */
typedef struct
{
    const uint8_t *data; /**< 数据首地址（仅本次回调有效） */
    uint32_t length;     /**< 字节数 */
    uint32_t reserved;   /**< 预留，恒 0 */
} dmcan_uart_chunk_t;

/**
 * @brief UART 线参数（对应 CDC 的 line coding）。
 *
 * @note `baud_rate = 0` 表示**不改**线参数（沿用设备当前配置）；
 *       默认（自动打开时用）是 115200 8N1。要和对端 UART 设备一致，改这里。
 * @note 固件对超范围波特率是"拒绝并保留上一次有效配置"（见 app 侧
 *       `tests/USB2UART_PERF_TEST.md` §3），所以下发失败不代表链路坏了。
 */
typedef struct
{
    uint32_t baud_rate; /**< 波特率；0 = 不改 */
    uint8_t data_bits;  /**< 5~8，默认 8 */
    uint8_t parity;     /**< 0=无 1=奇 2=偶，默认 0 */
    uint8_t stop_bits;  /**< 1 或 2，默认 1 */
    uint8_t reserved;   /**< 预留，恒 0 */
} dmcan_uart_config_t;

/** @brief UART 统计（只给计数，不产生逐字节事件） */
typedef struct
{
    uint64_t rx_bytes;   /**< 已投递给用户的字节数 */
    uint64_t tx_bytes;   /**< 已写入端口的字节数 */
    uint64_t rx_dropped; /**< 用户回调来不及/队列满而丢掉的字节数 */
    uint32_t errors;     /**< 串口层错误次数（含端口掉线） */
    uint8_t is_open;     /**< 端口当前是否打开 */
    uint8_t reserved[3]; /**< 预留，恒 0 */
} dmcan_uart_stats_t;

/*
 * 订阅事件（零拷贝：`frame` / `error` 指向 SDK 内部缓冲，**只在本次回调期间有效**，
 * 要留存请自己拷贝；两项不可能同时非 NULL）。
 */
typedef struct
{
    dmcan_event_flag_t type;            /**< 本次事件类型（单个位） */
    uint8_t channel;                    /**< 通道号 */
    uint8_t reserved[3];                /**< 预留，恒 0 */
    const dmcan_rx_frame_t *frame;      /**< 帧类事件（RX / TX 两类）有效，其余为 NULL */
    const dmcan_channel_error_t *error; /**< 错误事件有效，其余为 NULL */
    const dmcan_uart_chunk_t *uart;     /**< `DMCAN_EVENT_UART_RX` 有效，其余为 NULL */
    void *reserved2[1];                 /**< 预留，恒 NULL（占位与旧版一致，以后再加负载不用改签名） */
} dmcan_event_t;

/**
 * @brief 订阅回调。
 *
 * 使用约定（务必遵守）：
 *   1. **同步执行在该设备的通知线程上**（每设备一条），必须快速返回；不要在回调里做
 *      落盘、网络、长时间加锁这类重活，否则会拖慢该设备的全部事件；
 *   2. 同一设备的回调**串行**、不会并发；不同设备是不同线程，可能并发；
 *   3. `event` 与它指向的 `frame`/`error` **只在本次回调期间有效**；
 *   4. 回调里可以调 `dmcan_device_subscribe()` / `dmcan_device_unsubscribe()`（从下一批生效）；
 *   5. **不要在回调里 `dmcan_device_close()`**：会自锁（SDK 会直接拒绝并置
 *      `DMCAN_ERR_REENTRANT`，不会真的关设备）。
 */
typedef void (*dmcan_event_callback)(dmcan_device_handle *dev_handle, const dmcan_event_t *event, void *user_data);

/** 订阅配置 */
typedef struct
{
    uint32_t events;               /**< 事件位图，见 `dmcan_event_flag_t`；0 = 参数错 */
    uint32_t channel_mask;         /**< 关心的通道，见 `DMCAN_CHANNEL_MASK_ALL` */
    dmcan_event_callback callback; /**< 事件回调；不可为 NULL */
    void *user_data;               /**< 原样回传给 `callback`，SDK 不解释也不释放 */
} dmcan_subscribe_config_t;

/*
 * 订阅句柄（不透明）：由 SDK 持有，用户只把它传回给 `dmcan_device_unsubscribe()`。
 * 不要解引用、不要 free、不要跨设备使用（跨设备会返回 false 并置 `DMCAN_ERR_INVALID_ARG`）。
 */
typedef struct dmcan_subscription dmcan_subscription_t;
typedef struct dmcan_log_subscription dmcan_log_subscription_t;

/*
 * 日志回调：SDK 每产生一条日志就同步调用一次，用户可据此自己落盘。
 *
 * level 取值见 dmcan.h 的日志级别说明；seq 是进程内单调递增序号（跨线程按它排序才是确定顺序）；
 * line 是已格式化的整行文本（不含换行），**指针只在本次回调内有效**；
 * user_data 是注册时传入的值，原样回传。
 *
 * 使用约定：回调**同步执行在产生日志的那条线程上**，不要在回调里做重活（要落盘就自己攒队列
 * 由自己的线程写）；也不要在回调里再调 SDK 的日志接口（内部会丢弃重入日志）。
 */
typedef void (*dmcan_log_callback)(int level, unsigned long long seq, const char *line, void *user_data);

/*
 * 发送聚合档位（`dmcan_device_set_send_profile()`）。
 *
 * 发送路径会把同一小段时间内的帧**攒进一笔 USB 提交**（减少 USB 笔数；设备侧每笔 OUT 都有
 * 固定开销，一帧一笔在 8k 帧/s 会把设备主循环挤死，实测上传塌陷、echo 丢 91%）。
 * 代价是"连续数据流会被攒成突发"——就是用户反馈的"阶梯状"。两档：
 *
 * | 档位 | 目标批量 | 窗口上限 | 低速（<2000 帧/s） |
 * | --- | --- | --- | --- |
 * | `DMCAN_SEND_PROFILE_THROUGHPUT`（默认） | 8 帧/笔 | 1 ms | 窗口 100 µs，基本一帧一笔 |
 * | `DMCAN_SEND_PROFILE_LOW_LATENCY` | — | **恒 100 µs** | 同上 |
 *
 * @note 帧率 ≥2000 fps 时窗口按 `目标批量 ÷ 实测帧率` 放大（再夹到上表的上限）。
 * @note 延迟优先档把窗口钉死在 100 µs，于是 `1 / 100 µs` 也把 USB 笔数**封在 1 万笔/秒**
 *       （帧率再高只会"一笔多装几帧"），不会退化成"每帧一笔"。
 * @note 真机实测（5 kHz 连续流）：端到端 p50 **2107 µs → 1303 µs**、总线成簇 87% → 69%，
 *       0 丢帧；8 kHz / 20 kHz 也零丢帧（20 kHz 总线完全均匀）。进程 CPU 24.7% → 27.2%。
 *       换档前仍建议用 `tools/subscribe_perf_probe.exe --profile low` 在自己的场景下量一遍。
 * @note 延迟优先档**每笔装得更少、USB 笔数更多**，设备侧压力上升；换之前请用
 *       `tools/subscribe_perf_probe.exe` 在自己的场景下量过（总线成簇率、端到端延迟、
 *       `tx_dropped`/`echo` 计数）。
 */
typedef enum
{
    DMCAN_SEND_PROFILE_THROUGHPUT = 0,  /**< 吞吐优先（默认，等价于历史行为） */
    DMCAN_SEND_PROFILE_LOW_LATENCY = 1  /**< 延迟优先：攒批更小、帧到得更快 */
} dmcan_send_profile_t;

/*
 * 错误码：用 `dmcan_get_last_error()` 读取（**线程局部**，多线程各自独立）。
 *
 * 目前只区分"API 层能自己判定的原因"（参数/状态）；`DMCAN_ERR_IO` 是"与设备交互失败"的
 * 兜底码——USB 超时 / 固件 NACK / 队列满等更细的原因会随分层重构逐步细分，届时新增取值，
 * 已有取值保持不变（枚举尾部追加）。
 */
typedef enum
{
    DMCAN_OK = 0,               /**< 上一次调用成功 */
    DMCAN_ERR_INVALID_ARG,      /**< 入参非法（空指针、通道号越界等） */
    DMCAN_ERR_NOT_OPEN,         /**< 设备尚未 open（或已 close） */
    DMCAN_ERR_DEVICE_NOT_FOUND, /**< 设备索引越界 / 当前没有设备 */
    DMCAN_ERR_NO_MEMORY,        /**< 内存不足 */
    DMCAN_ERR_IO,               /**< 与设备交互失败（USB 超时 / 固件拒绝，细分见后续重构） */
    DMCAN_ERR_UNSUPPORTED,      /**< 设备或固件不支持该能力 */
    DMCAN_ERR_INTERNAL,         /**< 内部状态异常（不该发生） */
    DMCAN_ERR_REENTRANT         /**< 在 SDK 自己的回调线程上调用了会阻塞/自锁的接口（调用被拒绝，
                                     设备状态未变；请回到自己的线程再调用） */
} dmcan_status_t;

/*
 * 通道总线状态快照（`dmcan_device_get_bus_state`）。字段与固件 `CMD_GET_BUS_STATE`(27)
 * 的 5 字节应答一一对应，也与错误记录里的同名字段同语义。
 */
#pragma pack(push, 1)
typedef struct
{
    uint8_t bus_state; /**< 0=error active 1=error warning 2=error passive 3=bus-off */
    uint8_t tec;       /**< 发送错误计数（1:1，超过 255 会被固件截断到 255） */
    uint8_t rec;       /**< 接收错误计数 */
    uint8_t credits;   /**< 当前可用发送额度 */
    uint8_t in_flight; /**< 在途未确认的帧数 */
} dmcan_bus_state_t;
#pragma pack(pop)

/*
 * 通道统计快照（`dmcan_device_get_channel_stats`）。
 *
 * 值来自固件的**周期上报**（每秒一条），不是主动查询，所以最多旧 1 秒；字段与协议层
 * `dmcan_channel_stats_payload_t` 逐字段对齐（30B，v2 起含 `tx_failed`）。
 *
 * 后半段（`drop_flags` 之后）来自另外两条周期记录：`0x05`（接收/echo 侧细分）与
 * `0x07`（发送侧细分）。它们的作用是把 `rx_dropped` / `tx_dropped` 这两个合计拆到具体原因，
 * 例如"是设备内部写不进缓冲"还是"上位机没把数据读走"。两条记录各自独立到达，
 * 所以用 `drop_flags` 标明哪一组已经有值——**位为 0 时对应字段恒 0，那不是"没丢过帧"**。
 *
 * 对账关系（同一窗口）：`rx_dropped_queue + rx_dropped_msgc == rx_dropped 增量`；
 * `tx_dropped_no_credit + tx_dropped_queue_full + tx_dropped_submit_fail +
 * tx_dropped_ch_disabled == tx_dropped 增量`。
 */
/* 丢包细分的有效位（`dmcan_channel_stats_t::drop_flags`） */
#define DMCAN_DROP_FLAG_RX_ECHO (0x01) /**< bit0：`0x05`（接收/echo 侧细分）已收到 */
#define DMCAN_DROP_FLAG_TX (0x02)      /**< bit1：`0x07`（发送侧细分）已收到 */

#pragma pack(push, 1)
typedef struct
{
    uint32_t tx_admitted;      /**< 受理进发送队列的帧数（含后续被丢弃/失败的） */
    uint32_t tx_completed;     /**< 真正发送完成的帧数 */
    uint32_t tx_dropped;       /**< 因队列或额度不足丢弃的帧数 */
    uint32_t tx_failed;        /**< 发送尝试已结束但没上总线的帧数（一次性发送时最明显） */
    uint32_t rx_received;      /**< 收到的帧数 */
    uint32_t rx_dropped;       /**< 接收侧丢弃的帧数 */
    uint16_t sw_tx_fifo_level; /**< 软件发送队列当前深度 */
    uint16_t hw_tx_fifo_level; /**< 硬件发送 FIFO 当前深度 */
    uint8_t enabled;           /**< 该通道当前是否使能 */
    uint8_t reserved;          /**< 对齐保留，恒 0 */

    uint8_t drop_flags;        /**< 丢包细分有效位，见 `DMCAN_DROP_FLAG_*`；0 = 两组都还没收到 */
    uint8_t reserved2[3];      /**< 对齐保留，恒 0 */
    /* 以下 5 项来自 `0x05`（累计值，需 `DMCAN_DROP_FLAG_RX_ECHO` 置位才有效） */
    uint32_t rx_dropped_queue;      /**< 接收队列满丢弃的帧数（常见于上位机读得不够快） */
    uint32_t rx_dropped_msgc;       /**< 接收帧写不进 msgc 丢弃的帧数（设备内部资源） */
    uint32_t echo_dropped_queue;    /**< echo 待回传队列满丢弃的条数 */
    uint32_t echo_dropped_msgc;     /**< echo 写不进 msgc 丢弃的条数 */
    uint32_t echo_sent;             /**< 已写入 msgc（将随 `0x81` 上传）的 echo 条数 */
    /* 以下 4 项来自 `0x07`（累计值，需 `DMCAN_DROP_FLAG_TX` 置位才有效；四项之和 == tx_dropped 增量） */
    uint32_t tx_dropped_no_credit;  /**< 到达时 credit 耗尽，本帧不受理 */
    uint32_t tx_dropped_queue_full; /**< 到达时软件发送队列满，本帧不受理 */
    uint32_t tx_dropped_submit_fail;/**< 提交 MCAN 发送 FIFO 失败 */
    uint32_t tx_dropped_ch_disabled;/**< 通道已关闭（`CMD_DISABLE_CH` / 看门狗咬） */
} dmcan_channel_stats_t;
#pragma pack(pop)

/* 热插拔事件类型 */
typedef enum
{
    DMCAN_HOTPLUG_ATTACHED = 0, /**< 设备接入 */
    DMCAN_HOTPLUG_DETACHED      /**< 设备移除 */
} dmcan_hotplug_event_t;

/*
 * 热插拔事件里的设备描述。
 *
 * 刻意**不给句柄**：拔线时句柄可能正被其它线程使用，给出去就是共享生命周期问题。
 * 要拿句柄请自己走 `dmcan_find_devices()` + `dmcan_device_get()`——那条路径不会让
 * 已有句柄失效（见 `dmcan_find_devices()` 的说明）。
 *
 * **也不带序列号**：读序列号要打开设备，而回调跑在 libusb 事件线程上，不能在那里做
 * 这种慢操作；序列号在设备表刷新后从会话里取。
 */
typedef struct
{
    dmcan_device_type_t type; /**< 设备型号（按描述表判定；不认识的设备不会报事件） */
    int vendor_id;            /**< USB VID */
    int product_id;           /**< USB PID */
    uint8_t bus_number;       /**< 总线号 */
    uint8_t port_number;      /**< 端口号（本机后端拿不到时恒 0） */
} dmcan_hotplug_device_t;

/*
 * 热插拔回调。
 *
 * 使用约定（务必遵守）：
 *   1. **同步执行在 libusb 事件线程（或轮询线程）上**，必须快速返回；不要在回调里
 *      做打开设备、枚举、落盘这类慢操作，否则会拖慢所有设备的传输完成处理；
 *   2. **不要**在回调里调用 `dmcan_context_unregister_hotplug()`：注销要等同线程上的
 *      回调返回，而回调就在那条线程上，会死锁；
 *   3. `device` 指针只在本次回调内有效，需要保留请自己拷贝；
 *   4. 回调里调用 context/句柄的其它接口是允许的（内部不会持锁调用本回调）。
 */
typedef void (*dmcan_hotplug_callback)(dmcan_context *ctx, dmcan_hotplug_event_t event,
    const dmcan_hotplug_device_t *device, void *user_data);

/*
 * 批量/递增发送的配置（`dmcan_device_send_can_bulk`）。
 *
 * 这是**主机侧配置**，不参与协议线格式，所以没有按 1 字节对齐打包。
 *
 * 语义（实现由离线用例钉住，见 tests/test_bulk_send_plan.cpp）：
 *   - `id_inc` 为 true 时，每帧 ID 递增 `step_id`，发到 **`stop_id`（不含）** 为止；
 *     即发送区间是 `[can_id, stop_id)`；
 *   - `id_inc` 为 false 时 `step_id`/`stop_id` **被忽略**，按周期重复同一帧；
 *   - `data_inc` 为 true 时，payload 按 `dlen` 字节的**小端无符号整数逐帧 +1**，按字节回绕；
 *     与 ID 递增相互独立；
 *   - `send_times` 与 `stop_id` 谁先到谁停；`send_times = 0` 表示不限次数；
 *   - `interval_ms = 0` 表示"只发一帧"（等价单次发送）；否则按该周期发送；
 *   - `dlen = 0` 时没有数据可递增，`data_inc` 被忽略（记一条日志）；
 *   - `dlen` 会**自动向上取整到最近的 DLC**（9~12 → 线上 12 字节，尾部补 0）；
 *     "装不下"的组合（经典帧 `dlen > 8`、`dlen > 64`）直接返回 false 并置
 *     `DMCAN_ERR_INVALID_ARG`。
 */
typedef struct
{
    uint32_t interval_ms; /**< 发送间隔（毫秒）；0 = 只发一帧 */
    uint32_t send_times;  /**< 发送次数；0 = 不限（此时由 stop_id 决定何时停） */
    uint32_t can_id;      /**< 起始 ID */
    bool id_inc;          /**< 是否递增 ID */
    uint16_t step_id;     /**< ID 步进；仅在 id_inc 时有效 */
    uint32_t stop_id;     /**< 停止 ID（不含）；仅在 id_inc 时有效 */
    bool data_inc;        /**< 数据是否逐帧 +1（小端整数） */
    bool canfd;           /**< 帧类型：CAN FD */
    bool ext;             /**< 帧类型：扩展帧 */
    bool rtr;             /**< 帧类型：远程帧 */
    bool brs;             /**< 是否启用 BRS（仅 CAN FD 有意义） */
    uint8_t dlen;         /**< 数据长度（0~64） */
    const uint8_t *payload; /**< 数据；dlen 为 0 时可传 NULL */
} dmcan_bulk_send_config_t;

/*
 * 一帧待发送的报文（`dmcan_device_send_can_batch` 用）。
 *
 * 与 v1.1.0 的两段式（`fill_can_queue` 多次 + `can_queue_send` 一次）相比，数组版
 * **没有跨调用的残留状态**：一次调用交一批、一次提交，不存在"忘了 flush"这种状态。
 */
typedef struct
{
    uint32_t can_id;   /**< 帧 ID */
    bool canfd;        /**< 帧类型：CAN FD */
    bool ext;          /**< 帧类型：扩展帧 */
    bool rtr;          /**< 帧类型：远程帧 */
    bool brs;          /**< 是否启用 BRS（仅 CAN FD 有意义） */
    uint8_t dlen;      /**< 数据长度（0~64） */
    uint8_t payload[64]; /**< 数据；dlen 之外的字节被忽略 */
} dmcan_frame_t;

#ifdef __cplusplus
}
#endif

#endif // DMCAN_TYPES_H
