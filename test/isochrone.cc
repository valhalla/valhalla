#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "gurka/gurka.h"
#include "loki/worker.h"
#include "midgard/boost_geom_types.h"
#include "test.h"
#include "thor/worker.h"

#include <boost/geometry/algorithms/within.hpp>

#include <iostream>
#include <ranges>
#include <string>
#include <vector>

#ifdef ENABLE_GEOTIFF
#include <gdal_priv.h>
#endif

using boost::geometry::within;

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

using rp = rapidjson::Pointer;

namespace {

const auto cfg = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles",
                                   {{"service_limits.isochrone.max_locations", "2"}});

void check_coords(const rapidjson::Value& a, const rapidjson::Value& b) {
  EXPECT_NEAR(a.GetArray()[0].GetDouble(), b.GetArray()[0].GetDouble(), 0.00002);
  EXPECT_NEAR(a.GetArray()[1].GetDouble(), b.GetArray()[1].GetDouble(), 0.00002);
}

void test_iso_shape_equality(const rapidjson::GenericArray<false, rapidjson::Value>& actual_geom,
                             const rapidjson::GenericArray<false, rapidjson::Value>& expected_geom) {
  // different platforms can end up having some slightly different floating point wobble
  // to avoid failing tests we measure shape similarity and fail if its too far out of whack
  std::vector<PointLL> actual, expected;
  for (size_t j = 0; j < std::max(expected_geom.Size(), actual_geom.Size()); ++j) {
    if (j < actual_geom.Size()) {
      auto c = actual_geom[j].GetArray();
      actual.emplace_back(c[0].GetDouble(), c[1].GetDouble());
    }
    if (j < expected_geom.Size()) {
      auto c = expected_geom[j].GetArray();
      expected.emplace_back(c[0].GetDouble(), c[1].GetDouble());
    }
  }

#if defined(__aarch64__) || defined(__arm64__)
  EXPECT_TRUE(test::shape_equality(actual, expected, 200));
#else
  EXPECT_TRUE(test::shape_equality(actual, expected, 41));
#endif
}

void try_isochrone(loki_worker_t& loki_worker,
                   thor_worker_t& thor_worker,
                   const std::string& test_request,
                   const std::string& expected_json) {
  // compute the isochrone
  Api request;
  ParseApi(test_request, Options::isochrone, request);
  loki_worker.isochrones(request);
  auto response_json = thor_worker.isochrones(request);
  loki_worker.cleanup();
  thor_worker.cleanup();

  SCOPED_TRACE(response_json);

  // Parse isochrone json responses
  rapidjson::Document response, expected_response;
  response.Parse(response_json);
  expected_response.Parse(expected_json);

  // Same number of features
  auto feature_count = rp("/features").Get(expected_response)->GetArray().Size();
  EXPECT_EQ(rp("/features").Get(response)->GetArray().Size(), feature_count);

  // Check features are in the right order and look roughly the same
  for (size_t i = 0; i < feature_count; ++i) {
    // same metadata
    auto actual_properties = rp("/features/" + std::to_string(i) + "/properties").Get(response);
    auto expected_properties =
        rp("/features/" + std::to_string(i) + "/properties").Get(expected_response);
    EXPECT_TRUE((actual_properties && expected_properties) ||
                (!actual_properties && !expected_properties));
    if (expected_properties) {
      test::json_equality(*actual_properties, *expected_properties);
    }

    // same geom type
    std::string actual_type =
        rp("/features/" + std::to_string(i) + "/geometry/type").Get(response)->GetString();
    std::string expected_type =
        rp("/features/" + std::to_string(i) + "/geometry/type").Get(expected_response)->GetString();
    EXPECT_EQ(actual_type, expected_type);

    std::string coord_selector = "/features/" + std::to_string(i) + "/geometry/coordinates";
    // point is special
    if (expected_type == "Point") {
      check_coords(*rp("/features/" + std::to_string(i) + "/geometry/coordinates").Get(response),
                   *rp("/features/" + std::to_string(i) + "/geometry/coordinates")
                        .Get(expected_response));
    } // iteration required
    else if (expected_type == "LineString" || expected_type == "Polygon" ||
             expected_type == "MultiPoint") {
      // same geom appx
      uint32_t size = (actual_type == "LineString" || actual_type == "MultiPoint")
                          ? 1
                          : rp(coord_selector).Get(response)->GetArray().Size();
      for (uint32_t j = 0; j < size; ++j) {
        auto actual_geom =
            rp(coord_selector + (actual_type == "Polygon" ? "/" + std::to_string(j) : ""))
                .Get(response)
                ->GetArray();
        auto expected_geom =
            rp(coord_selector + (expected_type == "Polygon" ? "/" + std::to_string(j) : ""))
                .Get(expected_response)
                ->GetArray();
        test_iso_shape_equality(actual_geom, expected_geom);
      }
    } else { // MultiPolygon
      uint32_t actual_poly_size = rp(coord_selector).Get(response)->GetArray().Size();
      uint32_t expected_poly_size = rp(coord_selector).Get(expected_response)->GetArray().Size();

      EXPECT_EQ(actual_poly_size, expected_poly_size);

      // for each polygon
      for (uint32_t j = 0; j < actual_poly_size; ++j) {

        uint32_t actual_ring_count = rp(coord_selector).Get(response)->GetArray()[j].Size();
        uint32_t expected_ring_count = rp(coord_selector).Get(response)->GetArray()[j].Size();
        EXPECT_EQ(actual_ring_count, expected_ring_count);
        // test equality of each ring
        for (uint32_t k = 0; k < actual_ring_count; ++k) {
          auto actual_geom = rp(coord_selector + "/" + std::to_string(j) + "/" + std::to_string(k))
                                 .Get(response)
                                 ->GetArray();
          auto expected_geom = rp(coord_selector + "/" + std::to_string(j) + "/" + std::to_string(k))
                                   .Get(expected_response)
                                   ->GetArray();

          // tests exterior ring equality
          test_iso_shape_equality(actual_geom, expected_geom);
        }
      }
    }
  }
}

std::vector<PointLL> polygon_from_geojson(const std::string& geojson) {
  rapidjson::Document response;
  response.Parse(geojson);

  auto feature_count = rp("/features").Get(response)->GetArray().Size();
  for (size_t i = 0; i < feature_count; ++i) {
    std::string type =
        rp("/features/" + std::to_string(i) + "/geometry/type").Get(response)->GetString();

    if (type != "Point") {
      auto geom = rp("/features/" + std::to_string(i) + "/geometry/coordinates" +
                     (type == "Polygon" ? "/0" : ""))
                      .Get(response)
                      ->GetArray();
      std::vector<PointLL> res;
      res.reserve(geom.Size());
      for (size_t j = 0; j < geom.Size(); ++j) {
        auto coord = geom[j].GetArray();
        res.emplace_back(coord[0].GetDouble(), coord[1].GetDouble());
      }
      return res;
    }
  }
  return {};
}

TEST(Isochrones, Basic) {
  // Test setup
  loki_worker_t loki_worker(cfg);
  thor_worker_t thor_worker(cfg);
  GraphReader reader(cfg.get_child("mjolnir"));

  {
    SCOPED_TRACE("basic request 1 failed");
    const auto request =
        R"({"locations":[{"lat":52.078937,"lon":5.115321}],"costing":"auto","contours":[{"time":9.1}],"polygons":false,"generalize":55})";
    const auto expected =
        R"({"features":[{"properties":{"fill-opacity":0.33,"fillColor":"#bf4040","opacity":0.33,"fill":"#bf4040","fillOpacity":0.33,"color":"#bf4040","contour":9.1,"metric":"time"},"geometry":{"coordinates":[[5.042321,52.127328],[5.041287,52.12697],[5.041161,52.126096],[5.040249,52.126008],[5.040122,52.125135],[5.038321,52.124229],[5.038154,52.123103],[5.035757,52.121937],[5.034321,52.119335],[5.029718,52.120334],[5.027321,52.12207],[5.025528,52.122144],[5.025321,52.123105],[5.023541,52.123157],[5.022162,52.124937],[5.022291,52.123907],[5.023246,52.122862],[5.024983,52.122599],[5.025321,52.121687],[5.026948,52.121564],[5.030321,52.11812],[5.031738,52.118354],[5.031321,52.117214],[5.030321,52.117517],[5.030047,52.11421],[5.028541,52.112937],[5.029321,52.112615],[5.032072,52.116186],[5.034321,52.116434],[5.035321,52.115563],[5.037321,52.115677],[5.038321,52.114757],[5.042321,52.113929],[5.04477,52.112386],[5.047321,52.112088],[5.047504,52.11112],[5.049379,52.110995],[5.047711,52.109937],[5.053171,52.103787],[5.057006,52.103622],[5.058321,52.102514],[5.06139,52.102937],[5.060883,52.100375],[5.059321,52.099177],[5.056866,52.098937],[5.05795,52.098566],[5.057832,52.096937],[5.059321,52.096401],[5.059899,52.094937],[5.062669,52.096285],[5.063783,52.092937],[5.063321,52.092395],[5.062321,52.093261],[5.059321,52.093631],[5.057934,52.090323],[5.054833,52.088424],[5.051512,52.088128],[5.051321,52.089053],[5.051072,52.088185],[5.048754,52.08737],[5.04694,52.088317],[5.046687,52.08657],[5.044576,52.084937],[5.045321,52.084801],[5.048321,52.085511],[5.049057,52.082937],[5.046684,52.078573],[5.044029,52.077228],[5.044666,52.075591],[5.043709,52.076325],[5.042958,52.075937],[5.042651,52.072606],[5.041321,52.072508],[5.040321,52.073577],[5.03501,52.073626],[5.034321,52.078499],[5.028414,52.075937],[5.022321,52.075362],[5.021967,52.073583],[5.02617,52.072937],[5.027089,52.069705],[5.030321,52.071554],[5.037321,52.071094],[5.043321,52.069134],[5.047321,52.069256],[5.051321,52.067376],[5.054321,52.068228],[5.056321,52.06507],[5.059321,52.067212],[5.060321,52.066414],[5.0635,52.066116],[5.064045,52.063661],[5.065979,52.061937],[5.064157,52.058937],[5.064321,52.056054],[5.065321,52.058732],[5.065859,52.057475],[5.067741,52.057357],[5.068138,52.054754],[5.069986,52.058272],[5.073723,52.057937],[5.070993,52.057264],[5.069754,52.051503],[5.066165,52.051937],[5.066182,52.050798],[5.067806,52.050422],[5.070321,52.046925],[5.070489,52.047768],[5.072941,52.046557],[5.074321,52.047385],[5.075321,52.046359],[5.077321,52.046292],[5.078321,52.047272],[5.079321,52.046324],[5.080321,52.047249],[5.082321,52.046258],[5.084321,52.046217],[5.085321,52.047209],[5.086321,52.046384],[5.089321,52.046281],[5.090321,52.047243],[5.091321,52.046285],[5.093321,52.046294],[5.094027,52.04823],[5.098321,52.046241],[5.09915,52.049107],[5.101178,52.051079],[5.102321,52.050145],[5.104104,52.050153],[5.106321,52.050233],[5.107563,52.051179],[5.107901,52.048517],[5.111321,52.048246],[5.112321,52.046211],[5.113155,52.049102],[5.114081,52.049176],[5.114884,52.0465],[5.118321,52.046159],[5.120321,52.049157],[5.123321,52.049169],[5.124321,52.048185],[5.125321,52.049179],[5.137321,52.049241],[5.138321,52.050249],[5.141321,52.050273],[5.142321,52.048197],[5.144663,52.051937],[5.146321,52.051282],[5.147321,52.05227],[5.153321,52.052233],[5.15553,52.051146],[5.155729,52.048937],[5.156715,52.048542],[5.156054,52.053937],[5.159037,52.054937],[5.158321,52.055653],[5.155321,52.055672],[5.154104,52.057153],[5.157321,52.057235],[5.160013,52.058937],[5.157321,52.059637],[5.155848,52.058409],[5.152166,52.058782],[5.151448,52.063937],[5.149185,52.063801],[5.148593,52.065937],[5.14963,52.067627],[5.152321,52.068113],[5.158321,52.064142],[5.159115,52.064937],[5.158939,52.066555],[5.157196,52.066812],[5.153964,52.06958],[5.152204,52.06982],[5.151943,52.070937],[5.161804,52.079453],[5.163076,52.087937],[5.161705,52.090937],[5.163079,52.091937],[5.160753,52.092937],[5.160613,52.094229],[5.156863,52.097479],[5.157777,52.099393],[5.154858,52.100474],[5.153663,52.102937],[5.154321,52.103593],[5.156321,52.103359],[5.156892,52.104365],[5.161842,52.104937],[5.157321,52.105503],[5.156749,52.104508],[5.154045,52.104661],[5.148551,52.113937],[5.148559,52.117175],[5.146687,52.117303],[5.147321,52.119762],[5.149593,52.117937],[5.149478,52.119094],[5.146387,52.121003],[5.146353,52.122904],[5.147322,52.122935],[5.147321,52.124034],[5.147316,52.122941],[5.146302,52.122937],[5.145993,52.120264],[5.142638,52.119937],[5.146081,52.119697],[5.146024,52.116937],[5.147913,52.113345],[5.145043,52.113214],[5.142321,52.111362],[5.144917,52.109937],[5.142059,52.108198],[5.141321,52.106359],[5.140321,52.109125],[5.134,52.107257],[5.131323,52.107935],[5.129321,52.104612],[5.127463,52.105937],[5.125486,52.105102],[5.121321,52.106482],[5.116321,52.106378],[5.116321,52.110586],[5.113321,52.110283],[5.112766,52.111382],[5.108369,52.112985],[5.106828,52.112937],[5.109876,52.111492],[5.11084,52.108937],[5.108321,52.107485],[5.107321,52.10955],[5.106084,52.109173],[5.103321,52.110246],[5.101321,52.109546],[5.100321,52.110628],[5.094522,52.108736],[5.094321,52.107871],[5.092536,52.110152],[5.090321,52.110965],[5.079113,52.113144],[5.080159,52.110937],[5.079284,52.109937],[5.08185,52.108466],[5.083025,52.106641],[5.087321,52.107422],[5.089651,52.105606],[5.085321,52.103302],[5.084321,52.104615],[5.082878,52.104494],[5.0826,52.106216],[5.079349,52.105965],[5.078521,52.107137],[5.077321,52.107047],[5.077321,52.104012],[5.078321,52.104623],[5.078929,52.103937],[5.078321,52.103167],[5.077321,52.103618],[5.077321,52.102763],[5.078312,52.101928],[5.080321,52.102151],[5.081321,52.100616],[5.082321,52.101395],[5.082798,52.099937],[5.079321,52.099442],[5.078321,52.100513],[5.077321,52.098449],[5.076321,52.099213],[5.071321,52.098888],[5.071321,52.095681],[5.070164,52.09878],[5.061846,52.106462],[5.060234,52.10685],[5.05533,52.110937],[5.059321,52.112546],[5.062321,52.109518],[5.063768,52.109489],[5.063819,52.111937],[5.061808,52.112424],[5.057766,52.116937],[5.058695,52.118311],[5.056869,52.118485],[5.056386,52.119871],[5.058321,52.119782],[5.05897,52.118586],[5.06122,52.117836],[5.064004,52.117937],[5.062426,52.118042],[5.062321,52.1191],[5.060321,52.12016],[5.056281,52.119976],[5.054321,52.11823],[5.05036,52.118976],[5.050022,52.117235],[5.0475,52.117116],[5.047321,52.117766],[5.047321,52.11617],[5.049321,52.114042],[5.049475,52.114782],[5.052321,52.114542],[5.053321,52.115561],[5.055664,52.11528],[5.055612,52.113645],[5.053321,52.11155],[5.05109,52.111706],[5.049195,52.112811],[5.049212,52.113828],[5.046321,52.11377],[5.043321,52.115486],[5.041321,52.115474],[5.03587,52.118486],[5.038321,52.121582],[5.038623,52.119937],[5.039321,52.119889],[5.040321,52.120767],[5.041756,52.119937],[5.04205,52.120937],[5.038526,52.122142],[5.038414,52.122937],[5.038489,52.123768],[5.039656,52.123937],[5.041388,52.125869],[5.041476,52.126781],[5.042321,52.126912],[5.042321,52.127328]],"type":"LineString"},"type":"Feature"}],"type":"FeatureCollection"})";
    try_isochrone(loki_worker, thor_worker, request, expected);
  }

  {
    SCOPED_TRACE("basic request 2 failed");
    const auto request =
        R"({"locations":[{"lat":52.078937,"lon":5.115321}],"costing":"bicycle","costing_options":{"bicycle":{"service_penalty":0}},"contours":[{"time":15}],"polygons":true,"denoise":0.2})";
    const auto expected =
        R"({"features":[{"properties":{"fill-opacity":0.33,"fillColor":"#bf4040","opacity":0.33,"fill":"#bf4040","fillOpacity":0.33,"color":"#bf4040","contour":15.0,"metric":"time"},"geometry":{"coordinates":[[[5.116321,52.106987],[5.112321,52.107157],[5.111604,52.106653],[5.110321,52.106564],[5.109321,52.107208],[5.107321,52.107239],[5.102884,52.106373],[5.102321,52.105752],[5.101321,52.106387],[5.100321,52.106443],[5.097321,52.10545],[5.096321,52.105866],[5.094601,52.105657],[5.094321,52.105254],[5.094104,52.10572],[5.092519,52.105738],[5.091542,52.102937],[5.092724,52.100937],[5.092321,52.100091],[5.09084,52.100456],[5.090321,52.101367],[5.088837,52.101453],[5.088142,52.102758],[5.086321,52.10207],[5.085988,52.10127],[5.083454,52.100937],[5.08377,52.099937],[5.082116,52.099141],[5.076651,52.097937],[5.079497,52.097113],[5.079527,52.094937],[5.080211,52.093827],[5.08189,52.092937],[5.081321,52.092352],[5.079197,52.091937],[5.079547,52.091163],[5.081434,52.089937],[5.080108,52.089149],[5.079591,52.083937],[5.081534,52.08215],[5.081678,52.080937],[5.081321,52.079653],[5.078056,52.079672],[5.077861,52.081396],[5.079194,52.081937],[5.077321,52.083287],[5.075896,52.081361],[5.075789,52.079937],[5.077321,52.078252],[5.082321,52.078203],[5.082434,52.076823],[5.077321,52.077535],[5.075321,52.075538],[5.071321,52.075357],[5.070321,52.076122],[5.069284,52.075937],[5.070318,52.073934],[5.071321,52.073849],[5.072321,52.074446],[5.074321,52.07435],[5.074901,52.073517],[5.076516,52.073132],[5.07651,52.071747],[5.075321,52.071587],[5.07467,52.070937],[5.074895,52.069511],[5.075753,52.068937],[5.075855,52.065471],[5.081321,52.065183],[5.083945,52.062561],[5.086573,52.062189],[5.087142,52.060937],[5.089321,52.060545],[5.089711,52.061937],[5.088903,52.062937],[5.08897,52.064287],[5.090631,52.064247],[5.090951,52.063567],[5.092584,52.0632],[5.093321,52.062362],[5.094248,52.062937],[5.094321,52.063944],[5.094379,52.062995],[5.09544,52.063056],[5.095555,52.062171],[5.09644,52.062056],[5.096699,52.061315],[5.100599,52.058215],[5.10117,52.056786],[5.102321,52.056461],[5.102994,52.057263],[5.1036,52.056216],[5.106113,52.055937],[5.105321,52.054582],[5.10393,52.055327],[5.10235,52.054937],[5.10157,52.052937],[5.100708,52.052549],[5.100321,52.051656],[5.097321,52.051452],[5.096321,52.052331],[5.094702,52.052318],[5.094321,52.052738],[5.093822,52.051438],[5.095321,52.049876],[5.096321,52.049651],[5.097234,52.050023],[5.102321,52.050126],[5.102596,52.049661],[5.101648,52.048937],[5.102284,52.047937],[5.104321,52.047573],[5.105027,52.05023],[5.106321,52.050301],[5.107629,52.051245],[5.107915,52.050531],[5.109321,52.049619],[5.11165,52.049266],[5.112321,52.04711],[5.112972,52.049286],[5.117693,52.049937],[5.116069,52.051685],[5.113891,52.052507],[5.113723,52.053937],[5.113284,52.053937],[5.112649,52.051608],[5.111101,52.051717],[5.112064,52.052937],[5.112321,52.055195],[5.114321,52.055101],[5.115673,52.056585],[5.117321,52.056742],[5.118016,52.056632],[5.118321,52.055808],[5.120271,52.055887],[5.120321,52.055053],[5.120371,52.055987],[5.118897,52.056513],[5.119151,52.057106],[5.121835,52.057422],[5.123321,52.059126],[5.125321,52.059174],[5.126321,52.058181],[5.127321,52.05915],[5.127978,52.056594],[5.131635,52.056251],[5.132321,52.054321],[5.133321,52.05571],[5.136321,52.056155],[5.138412,52.056937],[5.13991,52.058347],[5.140735,52.058522],[5.141747,52.059511],[5.141943,52.060937],[5.142876,52.061381],[5.144321,52.063146],[5.149907,52.057523],[5.151616,52.057232],[5.152321,52.054013],[5.152321,52.058475],[5.150965,52.058581],[5.14797,52.062288],[5.149396,52.062937],[5.148849,52.063465],[5.148586,52.065937],[5.149858,52.067399],[5.151321,52.068196],[5.152321,52.068222],[5.153321,52.067247],[5.154565,52.067181],[5.156321,52.065393],[5.157704,52.06532],[5.158321,52.06458],[5.158677,52.064937],[5.158585,52.066201],[5.157012,52.066628],[5.155737,52.068353],[5.154321,52.068597],[5.15301,52.069937],[5.153321,52.070297],[5.157321,52.070479],[5.158321,52.069713],[5.158549,52.071165],[5.152321,52.071617],[5.151321,52.070689],[5.149132,52.072937],[5.151795,52.073463],[5.152321,52.075275],[5.153018,52.073634],[5.153624,52.073634],[5.154055,52.075202],[5.155685,52.075572],[5.155946,52.077311],[5.157407,52.07785],[5.156644,52.078937],[5.156913,52.079344],[5.158523,52.079735],[5.157909,52.081348],[5.159053,52.081669],[5.159321,52.081158],[5.159565,52.081692],[5.159321,52.082256],[5.157922,52.082538],[5.157321,52.084804],[5.153906,52.084522],[5.152064,52.084937],[5.152727,52.085343],[5.153278,52.084979],[5.153181,52.086076],[5.154778,52.086479],[5.155713,52.087937],[5.155633,52.089249],[5.154321,52.08988],[5.15267,52.089587],[5.150891,52.089937],[5.151321,52.090452],[5.153321,52.090661],[5.154321,52.090249],[5.153683,52.093299],[5.152522,52.094138],[5.151502,52.094118],[5.151824,52.095433],[5.153459,52.096075],[5.153321,52.096596],[5.153193,52.096064],[5.152321,52.096135],[5.151917,52.09734],[5.152839,52.097455],[5.153321,52.097012],[5.154727,52.097937],[5.15358,52.098196],[5.153321,52.098654],[5.152321,52.098304],[5.150908,52.098524],[5.151321,52.100317],[5.150321,52.100252],[5.149321,52.099387],[5.146321,52.099083],[5.145927,52.10033],[5.146686,52.101572],[5.147699,52.101937],[5.142604,52.10222],[5.142321,52.103202],[5.141997,52.10226],[5.141321,52.102112],[5.137084,52.102173],[5.13651,52.098937],[5.136629,52.098245],[5.137431,52.098047],[5.137405,52.096937],[5.136395,52.096937],[5.132321,52.099841],[5.131226,52.099842],[5.129321,52.101542],[5.128776,52.100481],[5.126321,52.100415],[5.125606,52.099651],[5.125014,52.09963],[5.12495,52.101307],[5.126416,52.101841],[5.127513,52.103129],[5.126321,52.103761],[5.124321,52.103402],[5.123321,52.104613],[5.119321,52.105377],[5.118321,52.106391],[5.117321,52.106272],[5.116321,52.106987]]],"type":"Polygon"},"type":"Feature"}],"type":"FeatureCollection"})";
    try_isochrone(loki_worker, thor_worker, request, expected);
  }

  {
    SCOPED_TRACE("basic request 3 failed");
    const auto request =
        R"({"locations":[{"lat":52.078937,"lon":5.115321}],"costing":"bicycle","costing_options":{"bicycle":{"service_penalty":0}},"contours":[{"time":15}],"show_locations":true})";
    const auto expected =
        R"({"features":[{"properties":{"fill-opacity":0.33,"fillColor":"#bf4040","opacity":0.33,"fill":"#bf4040","fillOpacity":0.33,"color":"#bf4040","contour":15.0,"metric":"time"},"geometry":{"coordinates":[[5.116321,52.106987],[5.112321,52.107157],[5.111604,52.106653],[5.110321,52.106564],[5.109321,52.107208],[5.107321,52.107239],[5.102884,52.106373],[5.102321,52.105752],[5.101321,52.106387],[5.100321,52.106443],[5.097321,52.10545],[5.096321,52.105866],[5.094601,52.105657],[5.094321,52.105254],[5.094104,52.10572],[5.092519,52.105738],[5.091542,52.102937],[5.092724,52.100937],[5.092321,52.100091],[5.09084,52.100456],[5.090321,52.101367],[5.088837,52.101453],[5.088142,52.102758],[5.086321,52.10207],[5.085988,52.10127],[5.083454,52.100937],[5.08377,52.099937],[5.082116,52.099141],[5.076651,52.097937],[5.079497,52.097113],[5.079527,52.094937],[5.080211,52.093827],[5.08189,52.092937],[5.081321,52.092352],[5.079197,52.091937],[5.079547,52.091163],[5.081434,52.089937],[5.080108,52.089149],[5.079591,52.083937],[5.081534,52.08215],[5.081678,52.080937],[5.081321,52.079653],[5.078056,52.079672],[5.077861,52.081396],[5.079194,52.081937],[5.077321,52.083287],[5.075896,52.081361],[5.075789,52.079937],[5.077321,52.078252],[5.082321,52.078203],[5.082434,52.076823],[5.077321,52.077535],[5.075321,52.075538],[5.071321,52.075357],[5.070321,52.076122],[5.069284,52.075937],[5.070318,52.073934],[5.071321,52.073849],[5.072321,52.074446],[5.074321,52.07435],[5.074901,52.073517],[5.076516,52.073132],[5.07651,52.071747],[5.075321,52.071587],[5.07467,52.070937],[5.074895,52.069511],[5.075753,52.068937],[5.075855,52.065471],[5.081321,52.065183],[5.083945,52.062561],[5.086573,52.062189],[5.087142,52.060937],[5.089321,52.060545],[5.089711,52.061937],[5.088903,52.062937],[5.08897,52.064287],[5.090631,52.064247],[5.090951,52.063567],[5.092584,52.0632],[5.093321,52.062362],[5.094248,52.062937],[5.094321,52.063944],[5.094379,52.062995],[5.09544,52.063056],[5.095555,52.062171],[5.09644,52.062056],[5.096699,52.061315],[5.100599,52.058215],[5.10117,52.056786],[5.102321,52.056461],[5.102994,52.057263],[5.1036,52.056216],[5.106113,52.055937],[5.105321,52.054582],[5.10393,52.055327],[5.10235,52.054937],[5.10157,52.052937],[5.100708,52.052549],[5.100321,52.051656],[5.097321,52.051452],[5.096321,52.052331],[5.094702,52.052318],[5.094321,52.052738],[5.093822,52.051438],[5.095321,52.049876],[5.096321,52.049651],[5.097234,52.050023],[5.102321,52.050126],[5.102596,52.049661],[5.101648,52.048937],[5.102284,52.047937],[5.104321,52.047573],[5.105027,52.05023],[5.106321,52.050301],[5.107629,52.051245],[5.107915,52.050531],[5.109321,52.049619],[5.11165,52.049266],[5.112321,52.04711],[5.112972,52.049286],[5.117693,52.049937],[5.116069,52.051685],[5.113891,52.052507],[5.113723,52.053937],[5.113284,52.053937],[5.112649,52.051608],[5.111101,52.051717],[5.112064,52.052937],[5.112321,52.055195],[5.114321,52.055101],[5.115673,52.056585],[5.117321,52.056742],[5.118016,52.056632],[5.118321,52.055808],[5.120271,52.055887],[5.120321,52.055053],[5.120371,52.055987],[5.118897,52.056513],[5.119151,52.057106],[5.121835,52.057422],[5.123321,52.059126],[5.125321,52.059174],[5.126321,52.058181],[5.127321,52.05915],[5.127978,52.056594],[5.131635,52.056251],[5.132321,52.054321],[5.133321,52.05571],[5.136321,52.056155],[5.138412,52.056937],[5.13991,52.058347],[5.140735,52.058522],[5.141747,52.059511],[5.141943,52.060937],[5.142876,52.061381],[5.144321,52.063146],[5.149907,52.057523],[5.151616,52.057232],[5.152321,52.054013],[5.152321,52.058475],[5.150965,52.058581],[5.14797,52.062288],[5.149396,52.062937],[5.148849,52.063465],[5.148586,52.065937],[5.149858,52.067399],[5.151321,52.068196],[5.152321,52.068222],[5.153321,52.067247],[5.154565,52.067181],[5.156321,52.065393],[5.157704,52.06532],[5.158321,52.06458],[5.158677,52.064937],[5.158585,52.066201],[5.157012,52.066628],[5.155737,52.068353],[5.154321,52.068597],[5.15301,52.069937],[5.153321,52.070297],[5.157321,52.070479],[5.158321,52.069713],[5.158549,52.071165],[5.152321,52.071617],[5.151321,52.070689],[5.149132,52.072937],[5.151795,52.073463],[5.152321,52.075275],[5.153018,52.073634],[5.153624,52.073634],[5.154055,52.075202],[5.155685,52.075572],[5.155946,52.077311],[5.157407,52.07785],[5.156644,52.078937],[5.156913,52.079344],[5.158523,52.079735],[5.157909,52.081348],[5.159053,52.081669],[5.159321,52.081158],[5.159565,52.081692],[5.159321,52.082256],[5.157922,52.082538],[5.157321,52.084804],[5.153906,52.084522],[5.152064,52.084937],[5.152727,52.085343],[5.153278,52.084979],[5.153181,52.086076],[5.154778,52.086479],[5.155713,52.087937],[5.155633,52.089249],[5.154321,52.08988],[5.15267,52.089587],[5.150891,52.089937],[5.151321,52.090452],[5.153321,52.090661],[5.154321,52.090249],[5.153683,52.093299],[5.152522,52.094138],[5.151502,52.094118],[5.151824,52.095433],[5.153459,52.096075],[5.153321,52.096596],[5.153193,52.096064],[5.152321,52.096135],[5.151917,52.09734],[5.152839,52.097455],[5.153321,52.097012],[5.154727,52.097937],[5.15358,52.098196],[5.153321,52.098654],[5.152321,52.098304],[5.150908,52.098524],[5.151321,52.100317],[5.150321,52.100252],[5.149321,52.099387],[5.146321,52.099083],[5.145927,52.10033],[5.146686,52.101572],[5.147699,52.101937],[5.142604,52.10222],[5.142321,52.103202],[5.141997,52.10226],[5.141321,52.102112],[5.137084,52.102173],[5.13651,52.098937],[5.136629,52.098245],[5.137431,52.098047],[5.137405,52.096937],[5.136395,52.096937],[5.132321,52.099841],[5.131226,52.099842],[5.129321,52.101542],[5.128776,52.100481],[5.126321,52.100415],[5.125606,52.099651],[5.125014,52.09963],[5.12495,52.101307],[5.126416,52.101841],[5.127513,52.103129],[5.126321,52.103761],[5.124321,52.103402],[5.123321,52.104613],[5.119321,52.105377],[5.118321,52.106391],[5.117321,52.106272],[5.116321,52.106987]],"type":"LineString"},"type":"Feature"},{"geometry":{"coordinates":[[5.115328,52.078939]],"type":"MultiPoint"},"properties":{"location_index":0,"type":"snapped"},"type":"Feature"},{"geometry":{"coordinates":[5.115321,52.078937],"type":"Point"},"properties":{"location_index":0,"type":"input"},"type":"Feature"}],"type":"FeatureCollection"})";
    try_isochrone(loki_worker, thor_worker, request, expected);
  }

  // multi-location
  {
    SCOPED_TRACE("basic request 4 failed");
    const auto request =
        R"({"costing":"auto","locations":[{"lon":5.086633,"lat":52.075911},{"lon":5.128852,"lat":52.109455}],"contours":[{"time":2}],"denoise":0,"generalize":100,"polygons":true})";
    const auto expected =
        R"({"features":[{"properties":{"fill-opacity":0.33,"fillColor":"#bf4040","opacity":0.33,"fill":"#bf4040","fillOpacity":0.33,"color":"#bf4040","contour":2.0,"metric":"time"},"geometry":{"coordinates":[[[[5.097852,52.083604],[5.092852,52.081719],[5.088852,52.083234],[5.084972,52.079334],[5.0815,52.079455],[5.091016,52.06929],[5.092793,52.070455],[5.090706,52.0716],[5.094852,52.072066],[5.094361,52.074945],[5.096971,52.074335],[5.0952,52.077455],[5.097139,52.078455],[5.097852,52.083604]]],[[[5.135852,52.111905],[5.12263,52.109676],[5.125079,52.104682],[5.129852,52.104372],[5.133852,52.107889],[5.137852,52.108703],[5.135852,52.111905]]]],"type":"MultiPolygon"},"type":"Feature"}],"type":"FeatureCollection"})";
    try_isochrone(loki_worker, thor_worker, request, expected);
  }

  // holes
  {
    SCOPED_TRACE("basic request 5 failed");
    const auto request =
        R"({"costing":"auto","locations":[{"lon":5.042799,"lat":52.093199}],"contours":[{"time":1}],"denoise":0,"generalize":0,"polygons":true})";
    const auto expected =
        R"({"features":[{"properties":{"fill-opacity":0.33,"fillColor":"#bf4040","opacity":0.33,"fill":"#bf4040","fillOpacity":0.33,"color":"#bf4040","contour":1.0,"metric":"time"},"geometry":{"coordinates":[[[5.045799,52.097112],[5.045609,52.096388],[5.045675,52.096199],[5.045666,52.096066],[5.04558,52.095417],[5.045566,52.095199],[5.045254,52.094743],[5.044799,52.094582],[5.044402,52.094802],[5.044162,52.095199],[5.044032,52.095432],[5.043799,52.095523],[5.0434,52.095598],[5.043189,52.095589],[5.042799,52.095719],[5.042367,52.09563],[5.042161,52.095561],[5.041799,52.095468],[5.041526,52.095471],[5.041071,52.095471],[5.040799,52.095473],[5.040601,52.095396],[5.040449,52.095199],[5.04029,52.094707],[5.04028,52.09468],[5.040116,52.094199],[5.040003,52.093994],[5.039799,52.093869],[5.039495,52.093895],[5.039042,52.094199],[5.038919,52.094319],[5.038799,52.094403],[5.038532,52.094465],[5.038065,52.094465],[5.037799,52.094581],[5.037713,52.094284],[5.037742,52.094199],[5.037735,52.094135],[5.037799,52.09359],[5.037965,52.093365],[5.037943,52.093199],[5.038467,52.092867],[5.038453,52.092544],[5.038577,52.092199],[5.038636,52.092036],[5.038799,52.091993],[5.039295,52.091695],[5.039569,52.091199],[5.039495,52.090895],[5.039616,52.090381],[5.039418,52.090199],[5.039721,52.090121],[5.039799,52.090099],[5.039973,52.090024],[5.040658,52.090058],[5.040799,52.089351],[5.040865,52.089265],[5.041008,52.089199],[5.041619,52.089019],[5.041799,52.089031],[5.041952,52.089045],[5.042508,52.088908],[5.042799,52.088923],[5.043036,52.088962],[5.043461,52.089199],[5.043598,52.089399],[5.043799,52.089721],[5.043892,52.090105],[5.043873,52.090199],[5.043987,52.090387],[5.044106,52.090891],[5.044509,52.090909],[5.044799,52.091045],[5.045005,52.090992],[5.045624,52.091199],[5.045103,52.091503],[5.045165,52.091832],[5.045054,52.092199],[5.045328,52.092669],[5.045799,52.09293],[5.046159,52.092838],[5.046485,52.092885],[5.046799,52.092622],[5.047059,52.092938],[5.047426,52.093199],[5.047577,52.09342],[5.047799,52.093565],[5.048184,52.093584],[5.048743,52.093254],[5.048799,52.093268],[5.048964,52.094033],[5.049326,52.094199],[5.048887,52.094287],[5.048799,52.09442],[5.048689,52.094308],[5.048046,52.094446],[5.047799,52.094385],[5.047419,52.094579],[5.047212,52.094612],[5.047126,52.094871],[5.04692,52.095199],[5.046927,52.095327],[5.046799,52.095576],[5.046439,52.095839],[5.046181,52.096199],[5.046142,52.096542],[5.045799,52.097112]],[[5.0442,52.0936],[5.044463,52.093199],[5.044483,52.092883],[5.044538,52.092459],[5.044556,52.092199],[5.04439,52.091791],[5.044441,52.091556],[5.044171,52.091571],[5.043799,52.091373],[5.0436,52.091397],[5.043104,52.091504],[5.042799,52.091534],[5.042392,52.091792],[5.042242,52.092199],[5.042445,52.092552],[5.042799,52.092699],[5.043132,52.092865],[5.043299,52.093199],[5.043448,52.093549],[5.043799,52.093738],[5.0442,52.0936]]],"type":"Polygon"},"type":"Feature"}],"type":"FeatureCollection"})";
    try_isochrone(loki_worker, thor_worker, request, expected);
  }
}

TEST(Isochrones, OriginEdge) {
  const std::string ascii_map = R"(
       a-b-c
     )";

  const gurka::ways ways = {
      {"abc", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 2000);
  auto map = gurka::buildtiles(layout, ways, {}, {}, "test/data/isochrones/origin_edge");

  std::string geojson;
  auto result = gurka::do_action(valhalla::Options::isochrone, map, {"b"}, "pedestrian",
                                 {{"/contours/0/time", "10"}}, {}, &geojson);
  std::vector<PointLL> iso_polygon = polygon_from_geojson(geojson);

  bg::polygon_ll_t polygon;
  for (const auto& p : iso_polygon) {
    boost::geometry::append(polygon.outer(), p);
  }
  EXPECT_EQ(within(map.nodes["b"], polygon), true);
  EXPECT_EQ(within(map.nodes["a"], polygon), false);
  EXPECT_EQ(within(map.nodes["c"], polygon), false);
}

TEST(Isochrones, ContoursOutOfBounds) {
  const std::string ascii_map = R"(
          c----d
         /
      a-b--------------f
    )";

  const gurka::ways ways = {
      {"ab", {{"highway", "primary"}}},
      {"bc", {{"highway", "primary"}}},
      {"cd", {{"highway", "primary"}}},
      {"bf", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  {
    auto map = gurka::buildtiles(layout, ways, {}, {},
                                 VALHALLA_BUILD_DIR "test/data/isochrones/service_limits",
                                 {
                                     {"service_limits.isochrone.max_distance_contour", "400"},
                                     {"service_limits.isochrone.max_time_contour", "200"},
                                 });

    // base case: within service limits, should succeed
    {
      auto result = gurka::do_action(valhalla::Options::isochrone, map, {"a"}, "pedestrian",
                                     {{"/contours/0/distance", "399"}}, {});
      result = gurka::do_action(valhalla::Options::isochrone, map, {"a"}, "pedestrian",
                                {{"/contours/0/time", "199"}}, {});
    }

    // distance exceeds service limits
    try {
      auto result = gurka::do_action(valhalla::Options::isochrone, map, {"a"}, "pedestrian",
                                     {{"/contours/0/distance", "500"}}, {});
      FAIL() << "Expected to throw";
    } catch (const valhalla_exception_t& e) {
      EXPECT_EQ(e.message, "Exceeded max distance: 400");
      EXPECT_EQ(e.code, 166);
    } catch (...) { FAIL() << "Expected valhalla_exception_t"; }

    // time exceeds service limits
    try {
      auto result = gurka::do_action(valhalla::Options::isochrone, map, {"a"}, "pedestrian",
                                     {{"/contours/0/time", "220"}}, {});
      FAIL() << "Expected to throw";
    } catch (const valhalla_exception_t& e) {
      EXPECT_EQ(e.message, "Exceeded max time: 200");
      EXPECT_EQ(e.code, 151);
    } catch (...) { FAIL() << "Expected valhalla_exception_t"; }
  }
}

TEST(Isochrones, LongEdge) {
  const std::string ascii_map = R"(
          c----d
         /
      a-b--------------f
    )";

  const gurka::ways ways = {
      {"ab", {{"highway", "primary"}}},
      {"bc", {{"highway", "primary"}}},
      {"cd", {{"highway", "primary"}}},
      {"bf", {{"highway", "primary"}}},
  };

  const auto layout = gurka::detail::map_to_coordinates(ascii_map, 100);
  auto map =
      gurka::buildtiles(layout, ways, {}, {}, VALHALLA_BUILD_DIR "test/data/isochrones/long_edge");

  std::string geojson;
  auto result = gurka::do_action(valhalla::Options::isochrone, map, {"a"}, "pedestrian",
                                 {{"/contours/0/time", "15"}}, {}, &geojson);
  std::vector<PointLL> iso_polygon = polygon_from_geojson(geojson);

  bg::polygon_ll_t polygon;
  for (const auto& p : iso_polygon) {
    boost::geometry::append(polygon.outer(), p);
  }
  EXPECT_EQ(within(map.nodes["a"], polygon), true);
  EXPECT_EQ(within(map.nodes["b"], polygon), true);
  EXPECT_EQ(within(map.nodes["c"], polygon), true);
  EXPECT_EQ(within(map.nodes["d"], polygon), true);
  EXPECT_EQ(within(map.nodes["f"], polygon), false);

  // check that b-f edges is visited and is partially within the isochrone
  auto interpolated = map.nodes["b"].PointAlongSegment(map.nodes["f"], 0.4);
  EXPECT_EQ(within(interpolated, polygon), true);
}

class IsochroneTest : public thor::Isochrone {
public:
  explicit IsochroneTest(const boost::property_tree::ptree& config = {}) : Isochrone(config) {
  }

  void Clear() {
    Isochrone::Clear();
    if (clear_reserved_memory_) {
      EXPECT_EQ(bdedgelabels_.capacity(), 0);
      EXPECT_EQ(mmedgelabels_.capacity(), 0);
    } else {
      EXPECT_LE(bdedgelabels_.capacity(), max_reserved_labels_count_);
      EXPECT_LE(mmedgelabels_.capacity(), max_reserved_labels_count_);
    }
  }
};

TEST(Isochrones, test_clear_reserved_memory) {
  boost::property_tree::ptree config;
  config.put("clear_reserved_memory", true);

  IsochroneTest isochrone(config);
  isochrone.Clear();
}

TEST(Isochrones, test_max_reserved_labels_count) {
  boost::property_tree::ptree config;
  config.put("max_reserved_labels_count_dijkstras", 10);

  IsochroneTest isochrone(config);
  isochrone.Clear();
}

#ifdef ENABLE_GEOTIFF

void check_raster_edges(size_t x, size_t y, uint16_t* data) {

  // make sure the outer "edges" are not 0
  for (size_t i = 0; i < y; ++i) {
    // if not in first or last row
    if (i != 0 || i != y - 1) {
      // just check first and last element in the row
      EXPECT_NE(data[i * y], 0);
      EXPECT_NE(data[i * y + x], 0);
      continue;
    }

    // else check the whole row
    for (size_t j = 0; j < x; ++j) {
      EXPECT_NE(data[i * y + j], 0);
    }
  }
}

TEST(Isochrones, test_geotiff_output_distance) {
  loki_worker_t loki_worker(cfg);
  thor_worker_t thor_worker(cfg);

  const auto request =
      R"({"costing":"auto","locations":[{"lon":5.042799,"lat":52.093199}],"contours":[{"distance":1}], "format": "geotiff"})";
  Api request_pbf;
  ParseApi(request, Options::isochrone, request_pbf);
  loki_worker.isochrones(request_pbf);
  std::string geotiff = thor_worker.isochrones(request_pbf);

  std::string name = "/vsimem/test_isogrid_geotiff_d.tif";
  std::vector<unsigned char> buffer(geotiff.length());
  std::copy(geotiff.cbegin(), geotiff.cend(), buffer.begin());
  auto handle =
      VSIFileFromMemBuffer(name.c_str(), buffer.data(), static_cast<int>(geotiff.size()), 0);
  auto geotiff_dataset = GDALDataset::FromHandle(GDALOpen(name.c_str(), GA_ReadOnly));
  int x = geotiff_dataset->GetRasterXSize();
  int y = geotiff_dataset->GetRasterYSize();
  GDALRasterBand* band = geotiff_dataset->GetRasterBand(1);
  std::vector<uint16_t> data_array(x * y);
  CPLErr err = band->RasterIO(GF_Read, 0, 0, x, y, data_array.data(), x, y, GDT_UInt16, 0, 0);
  double min_max[2];

  band->ComputeRasterMinMax(0, min_max);

  EXPECT_EQ(err, CE_None);
  EXPECT_NE(x, 0);
  EXPECT_NE(y, 0);
  EXPECT_EQ(static_cast<int>(min_max[0]), 0);
  EXPECT_EQ(static_cast<int>(min_max[1]), 1197);
  EXPECT_EQ(band->GetNoDataValue(), std::numeric_limits<uint16_t>::max());
  size_t array_size = x * y;

  check_raster_edges(x, y, data_array.data());

  // make sure there are some grid cells whose metric value is neither 0 nor the max
  bool no_intermediate_values = true;
  for (size_t i = 0; i < array_size; ++i) {
    if (data_array[i] > 0 && data_array[i] < min_max[1])
      no_intermediate_values = false;
  }
  EXPECT_EQ(no_intermediate_values, false);
  VSIFCloseL(handle);
}

TEST(Isochrones, test_geotiff_output_time) {
  loki_worker_t loki_worker(cfg);
  thor_worker_t thor_worker(cfg);

  const auto request =
      R"({"costing":"auto","locations":[{"lon":5.042799,"lat":52.093199}],"contours":[{"time":1}], "format": "geotiff"})";
  Api request_pbf;
  ParseApi(request, Options::isochrone, request_pbf);
  loki_worker.isochrones(request_pbf);
  std::string geotiff = thor_worker.isochrones(request_pbf);

  std::string name = "/vsimem/test_isogrid_geotiff_t.tif";
  std::vector<unsigned char> buffer(geotiff.length());
  std::copy(geotiff.cbegin(), geotiff.cend(), buffer.begin());
  auto handle =
      VSIFileFromMemBuffer(name.c_str(), buffer.data(), static_cast<int>(geotiff.size()), 0);
  auto geotiff_dataset = GDALDataset::FromHandle(GDALOpen(name.c_str(), GA_ReadOnly));
  int x = geotiff_dataset->GetRasterXSize();
  int y = geotiff_dataset->GetRasterYSize();
  GDALRasterBand* band = geotiff_dataset->GetRasterBand(1);
  std::vector<uint16_t> data_array(x * y);
  CPLErr err = band->RasterIO(GF_Read, 0, 0, x, y, data_array.data(), x, y, GDT_UInt16, 0, 0);
  double min_max[2];

  band->ComputeRasterMinMax(0, min_max);

  EXPECT_EQ(err, CE_None);
  EXPECT_GT(x, 0);
  EXPECT_GT(y, 0);
  EXPECT_EQ(static_cast<int>(min_max[0]), 0);
  EXPECT_EQ(static_cast<int>(min_max[1]), 2768);
  EXPECT_EQ(band->GetNoDataValue(), std::numeric_limits<uint16_t>::max());
  size_t array_size = x * y;

  check_raster_edges(x, y, data_array.data());

  // make sure there are some grid cells whose metric value is neither 0 nor the max
  bool no_intermediate_values = true;
  for (size_t i = 0; i < array_size; ++i) {
    if (data_array[i] > 0 && data_array[i] < min_max[1])
      no_intermediate_values = false;
  }
  EXPECT_EQ(no_intermediate_values, false);
  VSIFCloseL(handle);
}

// test request with two metrics
TEST(Isochrones, test_geotiff_output_time_distance) {
  loki_worker_t loki_worker(cfg);
  thor_worker_t thor_worker(cfg);

  const auto request =
      R"({"costing":"auto","locations":[{"lon":5.042799,"lat":52.093199}],"contours":[{"time":1},{"distance":2}], "format": "geotiff"})";
  Api request_pbf;
  ParseApi(request, Options::isochrone, request_pbf);
  loki_worker.isochrones(request_pbf);
  std::string geotiff = thor_worker.isochrones(request_pbf);

  std::string name = "/vsimem/test_isogrid_geotiff_td.tif";
  std::vector<unsigned char> buffer(geotiff.length());
  std::copy(geotiff.cbegin(), geotiff.cend(), buffer.begin());
  auto handle =
      VSIFileFromMemBuffer(name.c_str(), buffer.data(), static_cast<int>(geotiff.size()), 0);
  auto geotiff_dataset = GDALDataset::FromHandle(GDALOpen(name.c_str(), GA_ReadOnly));
  int x = geotiff_dataset->GetRasterXSize();
  int y = geotiff_dataset->GetRasterYSize();

  // time, distance
  std::array<int, 2> expected_max{2768, 1200};

  for (const auto b : std::views::iota(1, 2)) {
    GDALRasterBand* band = geotiff_dataset->GetRasterBand(b);
    std::vector<uint16_t> data_array(x * y);
    CPLErr err = band->RasterIO(GF_Read, 0, 0, x, y, data_array.data(), x, y, GDT_UInt16, 0, 0);
    double min_max[2];

    band->ComputeRasterMinMax(0, min_max);

    EXPECT_EQ(err, CE_None);
    EXPECT_NE(x, 0);
    EXPECT_NE(y, 0);
    EXPECT_EQ(static_cast<int>(min_max[0]), 0);
    EXPECT_EQ(static_cast<int>(min_max[1]), expected_max[b - 1]);
    EXPECT_EQ(band->GetNoDataValue(), std::numeric_limits<uint16_t>::max());
    size_t array_size = x * y;

    check_raster_edges(x, y, data_array.data());

    // make sure there are some grid cells whose metric value is neither 0 nor the max
    bool no_intermediate_values = true;
    for (size_t j = 0; j < array_size; ++j) {
      if (data_array[j] > 0 && data_array[j] < min_max[1])
        no_intermediate_values = false;
    }
    EXPECT_EQ(no_intermediate_values, false);
  }
  VSIFCloseL(handle);
}

TEST(Isochrones, test_geotiff_vertical_orientation) {
  loki_worker_t loki_worker(cfg);
  thor_worker_t thor_worker(cfg);

  const auto request =
      R"({"costing":"auto","locations":[{"lon":5.042799,"lat":52.093199}],"contours":[{"distance":1}], "format": "geotiff"})";
  Api request_pbf;
  ParseApi(request, Options::isochrone, request_pbf);
  loki_worker.isochrones(request_pbf);
  std::string geotiff = thor_worker.isochrones(request_pbf);

  std::string name = "/vsimem/test_isogrid_geotiff_d.tif";
  std::vector<unsigned char> buffer(geotiff.length());
  std::copy(geotiff.cbegin(), geotiff.cend(), buffer.begin());
  auto handle =
      VSIFileFromMemBuffer(name.c_str(), buffer.data(), static_cast<int>(geotiff.size()), 0);
  auto geotiff_dataset = GDALDataset::FromHandle(GDALOpen(name.c_str(), GA_ReadOnly));
  int y = geotiff_dataset->GetRasterYSize();
  double geoTransform[6];
  geotiff_dataset->GetGeoTransform(geoTransform);
  double topY = geoTransform[3] + 0 * geoTransform[4] + 0 * geoTransform[5];
  double bottomY = geoTransform[3] + 0 * geoTransform[4] + y * geoTransform[5];
  EXPECT_TRUE(topY > bottomY);

  VSIFCloseL(handle);
}
#endif

} // namespace

int main(int argc, char* argv[]) {

#ifdef ENABLE_GEOTIFF
  GDALRegister_GTiff();
#endif
  // user wants to try it
  if (argc > 1) {
    loki_worker_t loki_worker(cfg);
    thor_worker_t thor_worker(cfg);
    GraphReader reader(cfg.get_child("mjolnir"));
    Api request;
    ParseApi(argv[1], Options::isochrone, request);
    loki_worker.isochrones(request);
    std::cout << thor_worker.isochrones(request) << std::endl;
    return EXIT_SUCCESS;
  }
  // Silence logs (especially long request logging)
  logging::Configure({{"type", ""}});
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
