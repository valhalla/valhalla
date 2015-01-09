#include "tyr/handler.h"
#include "tyr/route_handler.h"
#include "tyr/locate_handler.h"
#include "tyr/nearest_handler.h"

#include <boost/python.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>


BOOST_PYTHON_MODULE(tyr_service) {
  boost::python::class_<valhalla::tyr::RouteHandler,
      boost::shared_ptr<valhalla::tyr::RouteHandler>,
      boost::noncopyable>("RouteHandler", boost::python::init<const boost::python::dict&>())
    .def("Action", &valhalla::tyr::RouteHandler::Action)
  ;
}
