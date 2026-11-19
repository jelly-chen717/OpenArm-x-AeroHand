#ifndef DMCAN_EXPORT_H
#define DMCAN_EXPORT_H

/* 两种交付形态共用这一份头：
   - 动态库：构建 SDK 自身时由构建系统定义 DEVICE_EXPORTS；使用方不定义（走 dllimport）。
   - 静态库（内部管理工具用）：构建时定义 DMDEVICE_STATIC，DEVICE_API 变空——静态库里
     既不能 dllexport 也不能 dllimport（同一 TU 里定义 dllimport 函数是错的）。
   使用方只需按自己拿到的形态对应的宏来用，不要手工定义这两个宏。 */
#if defined(DMDEVICE_STATIC)
#define DEVICE_API
#elif defined(_WIN32)
#ifdef DEVICE_EXPORTS
#define DEVICE_API __declspec(dllexport)
#else
#define DEVICE_API __declspec(dllimport)
#endif
#else
#define DEVICE_API __attribute__((visibility("default")))
#endif

#endif // DMCAN_EXPORT_H
