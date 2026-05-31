#include "filesystem_utils.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#include <windows.h>

#include <cstring>
#include <vector>
#endif

namespace valhalla::filesystem_utils {

void rename_replace(const std::filesystem::path& from,
                    const std::filesystem::path& to,
                    std::error_code& ec) {
#ifdef _WIN32
  // POSIX-semantics rename (Windows 10 1607+, guaranteed by the CMake version floor):
  // atomic and, like Linux, tolerant of readers holding the old target open.
  HANDLE handle = ::CreateFileW(from.wstring().c_str(), DELETE,
                                FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    ec.assign(static_cast<int>(::GetLastError()), std::system_category());
    return;
  }

  const std::wstring to_w = to.wstring();
  const size_t name_bytes = to_w.size() * sizeof(wchar_t);
  std::vector<char> buffer(sizeof(FILE_RENAME_INFO) + name_bytes, 0);
  auto* info = reinterpret_cast<FILE_RENAME_INFO*>(buffer.data());
  info->Flags = FILE_RENAME_FLAG_REPLACE_IF_EXISTS | FILE_RENAME_FLAG_POSIX_SEMANTICS;
  info->RootDirectory = nullptr;
  info->FileNameLength = static_cast<DWORD>(name_bytes);
  std::memcpy(info->FileName, to_w.c_str(), name_bytes);

  DWORD err =
      ::SetFileInformationByHandle(handle, FileRenameInfoEx, info, static_cast<DWORD>(buffer.size()))
          ? 0
          : ::GetLastError();
  ::CloseHandle(handle);

  // filesystems without POSIX-semantics rename (FAT/exFAT, some SMB shares) report
  // ERROR_INVALID_PARAMETER; fall back to a non-atomic replace there.
  if (err == ERROR_INVALID_PARAMETER) {
    err = ::MoveFileExW(from.wstring().c_str(), to.wstring().c_str(),
                        MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)
              ? 0
              : ::GetLastError();
  }
  ec = err ? std::error_code(static_cast<int>(err), std::system_category()) : std::error_code();
#else
  std::filesystem::rename(from, to, ec);
#endif
}

} // namespace valhalla::filesystem_utils
