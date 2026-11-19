/**
 * @file dmcan_version.h
 * @brief SDK 版本的唯一真源。
 *
 * 改版本只改这里：库内部（`dmcan_get_sdk_version()` 返回的整型值、`dmcan_print_version()`
 * 打印的字符串）与用户侧的头文件都从这几个宏来，不会再出现"库说自己 1.2.0、头文件写 1.1.0"。
 *
 * **权威在库里，不在这个头文件**：`dmcan_get_sdk_version()` 返回的是**库编译时**写死的值，
 * 用户改不动；这里的宏只是"这次编译所依据的声明"。两者对不上就是头文件与库不配套
 * （ABI 变更后会直接表现为结构体错位），用 `dmcan_check_version(DMCAN_VERSION_INT)` 一查就知道。
 *
 * 这里刻意用 `#error` 挡住外部定义（`-D` 覆盖或重复定义）：版本号属于库，
 * 不是调用方能"调"的参数；改了也只会让本文件与库对不上，改变不了库的行为。
 *
 * **递增规矩只对"对外发布过的版本"生效**：
 *   - ABI 不兼容（结构体布局、既有函数签名或语义变了）→ MAJOR +1，其余归零；
 *   - 新增接口（纯增量）→ MINOR +1，BUILD 归零；
 *   - 只修 bug、不动接口 → BUILD +1。
 * **还没对外发布过的版本不适用这套**：同一个未发布版本里可以随便加接口，版本号由发布者
 * 一次性定。**2.0 系列（`v2.0.0` / `v2.0.1`）至今只有内部 tag、没有对外分发**，所以本轮
 * （含"固件版本接口合并"这类 ABI 破坏性变更）按发布者口径抬 **MINOR** 定成 **`2.1.0`**，
 * 而不是抬 MAJOR；对外发布从 **`2.1.0`** 起算。具体内容见 `CHANGELOG.md`。

 */

#ifndef DMCAN_VERSION_H
#define DMCAN_VERSION_H

#include <stdint.h>

/* 挡住 -D 覆盖与重复定义：版本号不允许由外部定义 */
#if defined(DMCAN_VERSION_MAJOR) || defined(DMCAN_VERSION_MINOR) || defined(DMCAN_VERSION_BUILD) || \
    defined(DMCAN_VERSION_PATCH) || defined(DMCAN_VERSION_INT)
#error "DMCAN_VERSION_* 不允许外部定义：版本号是库的一部分，改这里不会改变库的行为（用 dmcan_check_version() 校验配套性）"
#endif

#define DMCAN_VERSION_MAJOR 2 /**< 主版本：发布后做 ABI 不兼容改动时 +1（规矩见文件头） */
#define DMCAN_VERSION_MINOR 1 /**< 次版本：新增接口时 +1（2.1.0：2.x 未对外分发，破坏性变更按发布者口径抬 MINOR） */
#define DMCAN_VERSION_BUILD 0 /**< 构建号：只修 bug、不动接口时 +1 */

#define DMCAN_VERSION_PATCH 0 /**< 补丁号：同构建号下的少量修正 */

/* 打包成 32 位整型，与 `dmcan_get_sdk_version()` 的返回值、以及 version_t 的字节序一致：
   低字节起依次是 patch / build / minor / major。 */
#define DMCAN_VERSION_INT                                                  \
    ((uint32_t)(((uint32_t)DMCAN_VERSION_MAJOR << 24) |                    \
                ((uint32_t)DMCAN_VERSION_MINOR << 16) |                    \
                ((uint32_t)DMCAN_VERSION_BUILD << 8) |                     \
                ((uint32_t)DMCAN_VERSION_PATCH)))

#endif /* DMCAN_VERSION_H */
