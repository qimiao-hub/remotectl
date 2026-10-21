#ifndef REMOTECTL_SOFTWARE_VERSION_H
#define REMOTECTL_SOFTWARE_VERSION_H

namespace remotectl {
// 登录报文0x01的软件标识：前两位为软件类型，后两位为版本号。
// 固定发送4个ASCII字符，不包含结尾NUL；修改后需重新编译程序。
inline constexpr char kSoftwareVersion[] = "0300";
static_assert(sizeof(kSoftwareVersion) == 5,
              "Software version must contain exactly four characters");
} // namespace remotectl

#endif
