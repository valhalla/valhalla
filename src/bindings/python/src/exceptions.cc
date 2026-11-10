#include "exceptions.h"
#include "module.h"

#include <nanobind/nanobind.h>

#include <exception>

namespace nb = nanobind;

namespace pyvalhalla {

void init_exceptions(nb::module_& m) {
  // Custom exception that exposes valhalla_exception_t fields to Python.
  // We create the type manually (instead of nb::exception) so that the translator
  // can populate structured attributes (code, http_code, etc.) on the instance.
  static PyObject* ValhallaError =
      PyErr_NewExceptionWithDoc("_valhalla.ValhallaError",
                                "Exception raised when a Valhalla operation fails.\n\n"
                                ":ivar code: Valhalla-internal error code.\n"
                                ":vartype code: int\n"
                                ":ivar message: Human-readable error message.\n"
                                ":vartype message: str\n"
                                ":ivar http_code: Corresponding HTTP status code.\n"
                                ":vartype http_code: int\n"
                                ":ivar http_message: Corresponding HTTP status message.\n"
                                ":vartype http_message: str\n",
                                PyExc_RuntimeError, nullptr);
  // don't increase refcount, it's static
  m.attr("ValhallaError") = nb::borrow(ValhallaError);

  // nanobind calls registered translators when a C++ exception escapes into Python.
  // The second arg (ValhallaError) is passed as payload to the lambda.
  // Other C++ exceptions (e.g. std::runtime_error) fall through to nanobind's
  // default translators.
  nb::register_exception_translator(
      [](const std::exception_ptr& p, void* payload) {
        try {
          std::rethrow_exception(p);
        } catch (const valhalla::valhalla_exception_t& e) {
          auto* type = reinterpret_cast<PyObject*>(payload);
          // Construct a ValhallaError instance: equivalent to `ValhallaError(e.what())` in Python
          nb::object exc = nb::steal(PyObject_CallFunction(type, "s", e.what()));
          if (exc.ptr()) {
            exc.attr("code") = nb::int_(e.code);
            exc.attr("message") = nb::str(e.message.c_str());
            exc.attr("http_code") = nb::int_(e.http_code);
            exc.attr("http_message") = nb::str(e.http_message.c_str());
            // Set the Python error indicator: raise exc
            PyErr_SetObject(type, exc.ptr());
          }
        }
      },
      ValhallaError);
}
} // namespace pyvalhalla
