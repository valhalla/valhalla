#include "valhalla/midgard/util.h"
#include "valhalla/midgard/pointll.h"
#include "test.h"

#include <string>

using namespace std;
using namespace valhalla::midgard;

namespace {

using perc_t = double;
using container_t = std::vector<std::pair<perc_t, perc_t> >;

bool appx_equal(const container_t& a, const container_t& b) {
  if(a.size() != b.size())
    return false;
  for(size_t i = 0; i < a.size(); ++i) {
    const Point2& x = static_cast<const Point2&>(a[i]);
    const Point2& y = static_cast<const Point2&>(b[i]);
    if(!x.ApproximatelyEqual(y))
      return false;
  }
  return true;
}

//need ostream operators for some of these types
std::string to_string(const std::vector<std::pair<double, double> >& points) {
  std::string out = "{";
  for(const auto& p : points) {
    out += "{" + std::to_string(p.first) + ", " + std::to_string(p.second) + "}";
  }
  out += "}";
  if(out.length() > 2)
    out.erase(out.end() - 3, out.end() - 1);
  return out;
}

void do_pair(const container_t& points, const std::string& encoded) {
  auto enc_answer = encode<container_t>(points);
  if(enc_answer != encoded)
    throw std::runtime_error("Simple polyline encoding failed. Expected: " + encoded + " Got: " + enc_answer);
  auto dec_answer = decode<container_t>(encoded);
  if(!appx_equal(dec_answer, points))
    throw std::runtime_error("Simple polyline decoding failed. Expected: " + to_string(points) + " Got: " + to_string(dec_answer));
  //cant run this thorough of a test due to accumulation of error
  /*if(encode<container_t>(decode<container_t>(encoded)) != encoded)
    throw std::runtime_error("Nested polyline encoding of a decoding failed");
  if(appx_equal(decode<container_t>(encode<container_t>(points)), points))
    throw std::runtime_error("Nested polyline decoding of an encoding failed");*/
}

void TestSimple() {
  /**
   * #to generate these a new test case install google polyline encoder for python:
   * sudo pip install gpolyencode
   * #then generate a test case with python:
   * python -c "import random; import gpolyencode; x = [ [round(random.random() * 180 - 90, 5), round(random.random() * 360 - 180, 5)] for a in range(0, random.randint(1,100)) ]; p = gpolyencode.GPolyEncoder().encode(x)['points']; print; print 'do_pair(' + str(x).replace('[','{').replace(']','}') + ', \"' + repr(p)[1:-1] + '\");'; print"
   */

  //check an easy case first just to be sure Point2/PointLL is working
  PointLL a(PointLL(1,2));
  if(encode<std::vector<PointLL> >({{-76.3002, 40.0433}, {-76.3036, 40.043}}) != "s}ksFhkupM|@dT") {
    throw std::runtime_error("Encoding of Point2/PointLL vector failed");
  }

  //note we are testing with higher precision to avoid truncation/roundoff errors just to make the test cases easier to generate
  do_pair({{-76.60025, 40.49437}}, "y`dvFp~orM");
  do_pair({{-81.86455, 32.02301}, {-47.2751, 177.93633}, {-65.47684, -6.48903}, {80.64511, 160.22116}, {-21.65714, -136.17079}, {-24.21342, -58.3363}, {-43.72562, -97.57677}, {16.64559, 76.62798}, {37.72706, 43.10825}}, "ynmbEldtrNguqyZawbrEnqsdb@z_bnBuqox^emzzZv``pw@`|koRs`azMvgrN|conFf~avBsjgf`@_gnoJfyqkEgnt_C");
  do_pair({{29.24635, 78.4994}}, "e|b~MudoqD");
  do_pair({{-45.72172, 27.85984}, {68.85349, 11.4313}, {40.92321, 116.33166}, {51.8195, -34.49974}, {31.79038, 132.07504}, {62.87121, 155.07137}, {20.7973, 117.77338}, {-4.91114, 52.84138}, {-80.57192, -83.24141}, {-35.16695, -66.65306}, {-24.65788, 67.62993}, {-74.46422, 26.17919}, {-50.77689, -62.89385}, {-45.3664, -139.27088}, {29.23634, 148.6219}, {-36.01982, -24.82529}, {-54.78416, -102.50572}, {-7.74692, -120.18365}, {-20.68256, -127.84536}, {-70.29023, 46.45027}, {-70.38103, -160.80128}, {-60.00434, -172.55985}, {-39.80232, -58.92514}, {63.70406, -112.54994}, {-7.52614, -41.60239}, {63.62614, -102.0144}, {18.87227, -37.54246}, {76.72405, -130.63901}, {-31.08523, -96.34625}, {-69.23605, -69.47891}, {30.35295, -120.1005}, {-88.89262, -47.4827}, {-42.9722, 154.83627}, {39.81484, -145.71364}, {27.83641, 114.99393}, {15.12587, -164.51484}, {-15.15319, -114.54826}, {-9.48589, 108.99042}, {-48.47169, 57.04111}, {1.93418, 177.72223}, {47.84336, -126.46884}, {10.68976, 108.64522}, {15.20771, 17.98434}, {-47.3135, -104.0049}, {12.0428, -90.88629}, {-47.8208, -132.80986}, {-30.28093, -101.23636}, {-24.44669, 96.95917}, {39.19122, 11.57052}, {-72.64948, 76.96498}, {7.89293, -110.53119}, {-61.44131, -124.4587}, {28.84163, 55.73208}, {-75.11697, -73.85547}, {17.22473, 34.20312}, {-14.10793, 170.80991}, {5.98697, -179.1232}, {12.11361, 145.14023}, {55.61778, -78.25513}, {-36.41204, 35.29354}, {48.89736, -154.67511}, {53.36265, 31.86117}, {-68.40494, 131.90514}, {25.84434, -110.78623}, {75.2159, 158.13543}, {27.10494, -47.26314}, {-70.34951, -134.59707}, {19.60138, 123.14479}, {-73.56791, 75.61906}}, "_k`iDv_avGjugcB_~hzTgjg_SfcniDhgrw[{doaAmcuw^~|fyBanjkCenu|DlwsbF|px_G~_ikKvd|{Cltq}X|nxlMe|fdBcdctGucrrXupc_Abz~{Frx~nHnat~OylqoClkdqMqv_`@{`d|u@czifM|ksa`@`jhmKt}byM`|oqB`v{jBw}a~Gtlwm@vn}mAubyf`@|~wmHdy}of@nvP`rwfAiui~@}gqtTsuhzB~qhfI{awvRe}_pLvcwqL`fvoJw|gqLsdohKt~cpGn|ewPsdb`JyxhpE~n_qS{o~bDtizgF|_~sHi~y~QgdfzLxcyvUqlzqe@sygwG|clix@}ihxNimfvp@bpbhAxp~ht@zoqlAcb~oHb{xwDw{jsi@s{qa@dja|Hfl}lF_pq_Vu{srHdis_y@ksewG{u_zk@~pwaFnejhPelqZv_qgVpdr|JifaoA{_hiJhe{~FnbklJkuu_Eew`jBaauxd@_orb@`ndhO}glcKkjcnKj{riT`jkwb@a}qjN|e_tA~ydfLkoxja@kk`fPda}uWfloyReeprSsnrrPm_x`Ybtf~D|hyvaAcxsyBmucz|@orkd@~{nri@a|ohGen`tTjqupP`gnfc@w~tgOwzoqb@acgZwyraRlvefV~sghm@_ag~Pkwjhr@g{ilH`|sdf@ntsdH`l`tOhayqQsecdp@ap_dPxjaaH`ctwP");
  do_pair({{-15.53885, -45.7488}, {-87.99271, 134.81781}, {-55.57838, -135.15261}, {81.78826, -38.81421}, {-52.36871, 45.42141}, {37.8654, 129.59886}, {-65.11989, -0.9595}, {-31.94747, 146.92637}, {35.19557, 12.58281}, {-64.21257, -63.39289}, {25.07945, -152.21426}, {-4.20052, -101.43325}, {-7.27603, 156.54674}, {-4.55068, 179.92153}, {-36.28176, -131.90092}, {-59.42427, -9.54753}, {-76.83488, -15.32229}, {-76.82168, 6.35475}, {46.86545, -65.13312}, {-13.11606, -88.88609}, {85.51166, -60.55316}, {36.11344, -40.65778}, {-85.87521, 149.61574}, {-71.98386, -29.9709}, {10.63218, -50.00946}, {23.34236, -104.13837}, {2.06619, -91.36492}, {-89.0105, -64.1214}, {62.97238, 73.18265}, {65.36177, -51.55572}, {43.60421, -70.64876}, {32.30238, -97.28242}, {5.33448, 62.1871}, {-6.17574, 179.48679}, {-64.7223, -41.1247}, {-15.70658, 6.64519}, {-64.38079, 167.524}, {12.57384, -114.04346}, {56.20931, -26.5548}, {80.25507, 151.23285}, {36.76444, 123.31472}, {-11.77555, 63.34836}, {-47.97595, -19.9084}, {69.61647, 163.29088}, {3.21875, -0.19615}, {39.22528, -32.48713}, {17.46684, 38.25412}, {-28.04298, -40.83794}, {-34.5019, 93.97423}, {-59.48558, -61.15085}, {-78.26262, 50.34457}, {-50.38747, 123.80487}, {63.02709, -23.9711}}, "~hfvGx|y}Ai|ama@rcfyLdrwnr@q|ydE_b_kQmlleYugcaO~oyqXa|w`OezvePvtz{W`iqsRu}re[s~miEf~}rX}zxxKb_vnMxsv}Qp|b}Ocz~_Pid}tHxvuqD}uqep@|twQmktmCmhsOjyenz@fnt`EwcxiVt_glCf{fb@hogiBoxhcCoqAfnisLqs|qV~f~oClcbmJyw|kDgf~xQaylxB|aolHoxihc@~{pgVlobga@{cxsA~whyBi}fwNt`kiIsmqlAay}lA`oz`C__heDhl{jPge`eY_ds~[v}ixVutqMnbpsBvoxcCj{paDlk~cAokil]jdrcDab}jUzageAxdoai@~zidJy`qbHgkdjHqs|t]xtahHrsput@meutMcs~tOupiiGycc|`@_mwqChwkiDlgmhGvd_mJ|mggHvad{Nnk}{Embd}a@sgvlUz`zd^vhgtKryadEyowzEyswnLfuxcCltvaNjswtGcoyuXfolf@vzxq\\~rnwCkmogTnkrqB{uj_MujciDxn}d[_hfsT");
  do_pair({{-21.99876, -1.86595}, {65.20593, -28.95221}, {25.15446, 0.1775}, {7.58781, -25.39538}, {-82.50103, 86.08717}, {27.68474, -176.59999}, {-1.99662, 52.64656}, {18.35328, 75.47717}, {-53.64825, 166.25227}, {84.68464, -65.08595}, {-46.03788, 124.73727}, {-31.92544, 58.35179}, {-5.78271, 108.52609}, {-81.57102, 24.55068}, {-13.60244, -48.29919}}, "dmkJfsgeCbhidDgdgsOukxpDrpmsFnua{Cp~ejB}|lgTfnzdPvaibq@ato_T}uevj@ncdtDibjjC{qe{Bko`iPpxmvLzf~bk@qcikYczqec@vvz|Wf|dtKwicuAktfqHa_q~Chmp_O|kqmMtns{Lsbz}K");
  do_pair({{66.48023, 142.09091}, {-83.09067, -31.97196}, {-59.92914, -85.73534}, {27.6751, 110.50725}, {-13.30534, 144.3244}, {87.79605, 125.31255}, {-13.31375, 3.30966}, {6.2234, 152.53296}, {-26.44057, -43.24853}, {0.47907, -74.43273}, {75.70072, -52.24577}, {-21.02541, 71.84482}, {49.05852, 76.55033}, {-74.83935, 36.44855}, {-59.25927, 120.67696}, {33.42697, 16.36381}, {-19.34156, -95.40514}, {79.21791, -25.84774}, {-50.48909, -148.47434}, {-85.01558, 42.93673}, {19.99389, -12.98517}, {-40.75998, -56.30109}, {-17.50394, 48.85642}, {-9.26468, -50.30697}, {-49.95242, 176.75196}, {11.14697, 111.02467}, {37.12012, 9.26275}, {-15.31676, 123.28632}, {61.08183, -148.36791}, {-38.99678, -106.54739}, {14.81998, -164.72891}, {53.48624, -145.1008}, {32.04177, -76.35032}, {-35.59652, 48.93058}, {-77.32175, 131.56946}, {-21.6581, 132.2413}, {11.99079, 106.60679}, {7.58201, -9.65202}, {-30.64146, -173.06528}, {-78.2657, -176.14755}, {-0.29352, -90.58355}, {-41.96173, -47.28886}, {74.69384, 12.76726}, {76.77565, 160.13635}, {-58.83791, -123.63408}, {-86.52392, 24.95332}, {54.86162, -63.38705}, {-7.78329, -68.0901}, {-49.61208, 32.26069}, {-20.7338, 93.93606}, {4.58431, -153.61695}, {76.27444, -155.58128}, {-25.81991, -81.07005}, {12.96994, -170.457}, {51.76011, 118.73211}, {-83.78267, 92.71166}, {80.67356, 179.97168}, {-19.11467, 89.72216}, {-85.60799, 120.61093}, {-64.98943, -23.85667}, {-34.47639, -86.51635}, {-49.38051, 30.62519}, {6.03036, -35.25185}, {-37.32907, -17.84905}, {-86.38651, 145.61495}, {25.86888, 32.0037}, {74.93306, -160.01443}}, "ecgbZmlwtK|ske`@ba|o[btcgIqvjlCeswld@oeuuOe|kmEv~byF`g`sBujahR`usgVf_chRsdxm[ezfvBhq}id@xtjfEhti}DwvhcDqklfCivbjMemktVhyjmQm`v[qgwjLdkwsFtxesVszaaOo~a~Adtt{R}wutP|zdiThjaaIwlpgLw{pxQhomkVvktvWg~goc@pmvqEzfitIet|_SnckgGt~xqJmqy`Sge}lCdzf|Qkfhq@imzhj@jyiwFpjdpKen|sJ~jblRe{o}Cif}vTnq`_I|m`yr@erhqMgag~FhryaRpqbbJwangIwrxvBc__kFoyrbL|j{aCs|s{Vhry{K_lkwNtmt}F_fbCyxvrItvm{Cq`klEpxqdUza|Yzskd^tohhFd_yQnrtaH_vfiOc}{zMy~fgGhii}FwupmJgh_gUy_nb[kruKd|~bu@v_vzXg~{i[pl~gDh~dzOsj}}Ybqu[tij}JmwncR|th~Fc~lwJghgoDhe}en@e}oyC`t~Ji~ptLe~weMthcnRlkq`PqcwkF}fadv@qewkFxby}CjehzXc~qsOkjwj^nzyePj{``Ry}o{Df~ytKniwpZ_az|B~em}JoqfyDse~iUv}}yAnraqK}leqIo~eiBlssgG_qud^~oljHhultTe{clThp~rc@czmjH");
  do_pair({{22.15968, -121.74934}, {36.71865, 90.28283}, {-76.64698, 64.63014}, {72.99198, -68.3195}, {-19.9686, -71.22584}, {30.74348, -5.67267}, {34.17338, 34.69123}, {-41.44118, 154.30724}, {-87.01174, -111.63206}}, "jdbfV_agfCapcmg@opzwAhhq{Cbv|rThvmjXojip[rsvPrjkvPkjboKouotHiqjuF{{|ScoayU`nolMrgdvq@|ncuG");
  do_pair({{-18.90401, 88.21983}, {70.91829, -124.95024}, {13.34816, -11.22116}, {-9.4033, -96.46758}, {45.95577, 144.22878}, {-58.36483, -20.23525}, {-60.01575, 116.77996}, {-1.27272, 100.55303}, {-88.63538, -163.22847}, {18.55161, 106.68361}, {-10.52036, 35.64569}, {-31.57875, 90.72262}, {54.09084, -179.71126}, {73.06491, -127.85302}, {35.14424, 167.59731}, {75.40938, 81.77964}, {-49.59798, -62.34892}}, "}lmyO`ekrB|watg@klfcPwucuThdk~IbuhgOrsziCg_b|l@ei{pId{xj^vbv{RaxgcYfmaIhi`bB{fpeJjy~hq@p_ftOoelnr@u}emS~qqpLxbmpDyedoI|}o_Cvbrqr@}i{iO_qo{H}zxrBqchjw@dkmfFlgxjOchwtFnbunZ~n~yV");
  do_pair({{70.48061, 95.62155}, {74.84481, -82.78123}, {44.12176, 144.99742}, {-83.32643, 17.62806}, {88.7132, -93.92446}, {41.61047, -66.98812}, {-15.72204, 110.73308}, {84.39213, -76.424}}, "uasfQyvdmLjh{_a@gksYq_gmj@`rozDnykhWdf{hWfrzgTun`y_@c_lcD`wn~Godv{`@dw||Ixbiub@qp`bR");
  do_pair({{14.46242, 52.04926}, {-26.18271, -165.60311}, {-72.79692, -108.00537}, {-48.33105, -83.59655}, {-36.64776, -103.80287}, {11.58542, -136.05612}, {29.27928, -150.32051}, {-79.89048, -54.69626}, {18.31496, 172.03533}, {-14.52716, 34.2364}, {-45.40556, -65.12207}, {67.65398, 35.48119}, {-24.86036, -136.05284}}, "{zt|HcugwAhfmoh@`oawF{pp~Ixio{Gcj~sCunitCnpizBq{hfAxmzcEkpkeHn_avAsy~jBsrsfQ~eiySmozfj@_wkvQhz`hYfnmgEl}l}Q~|m{Dka`eRc}`qTtv}u_@retsP");
  do_pair({{0.00416, 0.21896}, {0.21187, 0.1913}, {0.26889, 0.21005}}, "owi@}XzkDgqg@etBkcJ");
  do_pair({{0.83055, 0.16828}}, "wz_@}eaD");
  do_pair({{0.3668, 0.16416}, {0.59339, 0.55088}, {0.36359, 0.97223}, {0.98343, 0.42627}, {0.69262, 0.59437}, {0.33802, 0.3519}, {0.50652, 0.09935}, {0.43568, 0.02901}, {0.16688, 0.21628}, {0.63935, 0.92504}}, "_a_@osfA_pjAegk@mhqAf{k@fsiB_axBqy_@pxw@jjn@fgdAlip@c|_@rvLvyLmqc@~ns@wliC}g{A");
  do_pair({{0.85353, 0.12266}, {0.42553, 0.52523}, {0.52006, 0.22074}, {0.6737, 0.05832}, {0.1182, 0.07611}, {0.49973, 0.60776}}, "s}VqueDasmA~qrA`nz@ymQbv^g_]enBznkByyfBqoiA");
  do_pair({{0.92801, 0.38651}, {0.53295, 0.9549}, {0.55884, 0.46084}, {0.79716, 0.23435}, {0.31782, 0.50702}, {0.75745, 0.62159}, {0.81565, 0.92208}, {0.1393, 0.79799}, {0.49189, 0.29248}, {0.41221, 0.62125}, {0.26266, 0.6343}, {0.96519, 0.37615}, {0.38833, 0.21842}, {0.66702, 0.36709}, {0.28903, 0.96824}, {0.34385, 0.55213}, {0.76157, 0.39392}, {0.53174, 0.36369}}, "unjAagtDm_nBbdlAzn_By`Dpfk@opm@egt@zr|AakUuztAauy@wjJpfWdbcClvaBuzcAye_A~pNqpAte\\llq@yehCxx]jtoBe`\\ylu@eltBlyhAtgpAsuIx{]wqpA|{Dl{k@");
  do_pair({{-0.02247, -0.05852}, {0.22558, -0.02274}, {-0.01555, 0.29148}, {0.36503, -0.38705}, {0.10947, 0.14513}, {-0.2007, 0.46559}, {0.11071, 0.23225}, {-0.12726, -0.49291}, {-0.3762, -0.16376}, {-0.14745, 0.43701}, {0.46148, -0.33037}, {-0.00443, 0.30567}, {0.19057, -0.24104}, {-0.16804, 0.35654}, {0.06911, -0.19944}, {0.1821, 0.19466}, {0.46591, 0.26364}, {0.08704, 0.13526}, {-0.19553, -0.40819}, {-0.45347, -0.00998}, {0.25084, -0.42184}, {0.42069, 0.12764}, {-0.20883, -0.06028}, {-0.2616, 0.43711}, {0.06636, 0.17041}, {-0.28213, 0.06163}, {-0.13774, -0.22601}, {-0.28276, 0.17662}, {0.37572, -0.09245}, {0.37202, 0.38676}, {-0.41465, 0.27058}, {-0.33643, 0.03093}, {0.10492, -0.00819}, {0.11926, -0.42677}, {0.01055, -0.49845}, {-0.202, 0.16724}, {0.47641, -0.37119}, {-0.49069, 0.19545}, {-0.30342, 0.35461}, {-0.08564, -0.16813}, {0.21759, 0.42968}, {0.31061, 0.2205}, {0.20226, 0.23306}}, "vlJlkCs~Eimo@{j|@`bn@xocCsiiAc}fBf|p@{q}@pq{@jql@iy{@fslCjnm@eh_Axro@yitButk@b{tCy|uBgf{B|~yA|wiBwae@{usBh`eAzqkBeim@c~kAeaUcnLylv@jaX|~hApciB`ev@ywlAbkq@bmoA}phCgijBqd`@nuc@n}yBuc`BxhIzas@w`_AzfT`acAvdw@me[msmAji[tps@or_Car|AbVbuUtsxCxxm@{gNnsFmeuAbwpAsxA~~LlfTq_aCloh@ddhBaocCotmBj{{Dwa^kqc@bbeBepi@iwsBefz@jzg@kdQomAddT");
  do_pair({{-0.22425, -0.39183}, {0.38378, 0.00592}, {-0.18061, 0.10003}, {0.46866, -0.37526}, {0.47946, -0.01538}, {0.46096, 0.08755}, {-0.09106, -0.19655}, {0.30615, -0.43688}, {-0.03909, 0.00453}, {0.23839, 0.41771}, {0.17034, -0.07711}, {-0.44147, 0.02292}, {0.44295, 0.22162}, {0.17964, 0.07811}, {0.11855, -0.0248}, {-0.43225, 0.47054}, {-0.29875, 0.48948}, {0.09079, 0.14901}, {-0.21332, 0.27206}, {0.01766, -0.00848}, {0.02723, 0.45504}, {0.01489, -0.09384}, {0.1155, 0.23336}, {-0.28927, 0.00276}, {0.0525, -0.42008}, {-0.16193, 0.41477}, {-0.49805, -0.17974}, {0.44745, -0.20408}, {-0.40406, 0.32143}, {0.17083, 0.05995}, {-0.31817, -0.38772}, {-0.0358, -0.26725}, {-0.25421, -0.39911}, {-0.33031, 0.24712}, {0.0815, 0.03831}, {0.28679, 0.39804}, {0.19295, -0.4493}, {0.18557, -0.45465}, {-0.16927, -0.19152}, {-0.29579, 0.02072}, {0.16317, -0.36696}, {0.32473, 0.14214}, {-0.22843, 0.42688}, {-0.30822, -0.26349}, {0.0712, -0.20262}, {-0.22311, -0.03045}, {-0.26854, 0.07089}, {-0.39583, -0.04182}, {-0.12036, 0.10093}, {0.26003, -0.46725}, {-0.28637, -0.03047}, {0.35129, 0.15023}, {0.21197, -0.43292}, {0.48227, 0.17942}, {0.17417, 0.48049}, {0.31848, -0.18252}, {0.45591, 0.25038}, {0.2979, 0.0351}, {-0.24139, 0.35056}, {0.08149, -0.09246}, {-0.46029, 0.04881}, {0.40682, -0.06633}, {0.48781, 0.46001}, {0.35453, -0.37808}, {-0.11795, -0.14154}}, "|okApxj@}tlAewuBekQlfmBpy{A}x}BgheAobAibSrrBrnv@byjB`}m@oqlAyeuAtlbAkuoAgeu@rs_BhhLepRxnvB{xe@svkD|_[tlr@dbSx|J{v_BnqjBkuBkaY|naAsakAa`Wtkz@jxu@qbl@_pyA{z@nejBblA_|~@ysRf`l@z`nAvqqAcwaAy`bDd{h@tbsBvs`AbwCktwDmseB|heDfar@ahoB|lvAfo~A}oVycv@bwX`ti@}e}BrzMbxg@yloAkgeAabg@zndDniQl`@bm@qkr@vhdAomh@vuW~ujAosxAylbBwp^urv@f`kBxyeCtqNm{JkbiAas`@lnx@ixR|zGj_UpzWe{Zuxt@b~mBmhiA{htA~uiBkhb@kp{Bt{pBveZcrvBkxs@uxy@rd{@xn`C}d[spsA}yYn`i@pz]sr|@pihBzouA_a~@}qZbyhBrnUmjhDsxeBeyN`ubD~_Ykem@~g{A");
  do_pair({{0.16446, -0.20374}, {0.17675, -0.14692}, {-0.45127, 0.38007}, {-0.4412, 0.12956}, {-0.03543, -0.46103}, {0.13252, 0.38625}, {0.21882, -0.13666}, {0.31078, -0.47756}, {-0.20149, 0.10185}, {0.22546, 0.45115}, {-0.15274, -0.24138}, {0.30771, 0.04852}, {-0.37215, -0.06894}, {0.30563, -0.06334}, {0.07926, 0.43394}, {-0.00202, -0.27494}, {0.37189, 0.39752}, {0.07641, 0.40565}, {0.23898, -0.09151}, {-0.37925, 0.14613}, {0.00399, -0.33225}, {-0.33378, -0.47005}, {0.46418, 0.02027}, {0.21532, 0.469}, {0.45061, -0.12145}, {0.36292, -0.27092}, {0.21876, 0.15861}, {-0.05141, 0.46466}, {-0.04606, 0.18647}, {0.43057, -0.07357}, {0.33029, -0.1321}, {0.42462, 0.13134}, {-0.40924, 0.39276}, {0.10296, 0.37172}, {0.02983, 0.0503}, {0.47632, 0.2995}, {0.22172, 0.39419}, {0.35175, 0.05948}, {-0.37854, -0.15232}, {-0.44397, -0.34853}, {-0.23252, 0.34051}, {-0.25944, 0.03477}, {0.21356, 0.35665}, {-0.36126, -0.46567}, {-0.10415, -0.39468}, {-0.32892, -0.33643}, {-0.42049, 0.24614}, {-0.04436, 0.36247}, {0.32487, 0.14993}, {0.37662, -0.06131}, {-0.37975, 0.05587}, {-0.25628, 0.1504}, {-0.3639, -0.32263}, {-0.12365, -0.25974}, {-0.41876, 0.33837}, {0.49991, 0.10387}, {0.1119, -0.33496}, {-0.17086, -0.4687}, {0.35536, -0.16303}, {0.40591, -0.26593}, {-0.33183, 0.42948}, {0.21976, 0.06125}, {-0.1726, -0.08216}, {-0.27962, 0.34903}, {-0.05536, -0.22156}, {-0.43957, 0.37449}, {-0.27458, -0.10122}, {0.24696, -0.33437}}, "jxf@{b_@cbJykAu|eBbtyBt|o@}}@djrB_gnAondDwx_@dceBkzOrqaAw}PidpBt`cBcfcAmkrAhgfCvzhA{rw@y|xAb}UbxcC_b@ckcC_c`Bxek@nmiC~zN{ibC}_hAyq@vux@fb`Baw^glm@|vwBzl|AeziAf|Y~}`Aow~AgzzCqsvAjro@hirBq}l@de\\`cPq{rA~c[ywz@pws@tiu@m`@fxq@}a|AxlJvqRomr@qlQ{`r@rzaDnbCg`cBzw}@`hMoto@qevAynQfvp@|j`AukXxjh@hsmCfie@|wKoqeCqhh@zuz@fgDwz}@gk{Anr_DrgoBuzL}eq@akJx{j@axpBh{PavUymhAjoh@ubgAfgh@mbIk{UhvrCymQubWlk{Ar_T_hKq|m@gysBlsx@rxl@ulrDtutA`xjAzbYffv@muz@{weBbbS}zHiyfCzaoCl|fAmvjBh_[fskA}esAz{SdmnBsxj@ilsBh`jAd|{Aef_@dpl@szdB");

}

}

int main() {
  test::suite suite("encode_decode");

  // Test kilometer per degree longitude at a specified latitude
  suite.test(TEST_CASE(TestSimple));

  return suite.tear_down();
}
