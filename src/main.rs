use std::collections::BTreeMap;
use std::time::SystemTime;

use petgraph::Graph;
use petgraph::prelude::*;
use rayon::prelude::*;
use crate::types::{Coordinate, Map};

mod algorithms;
mod types;

const MAP_FILE_PATH: &'static str = "mad1.osm.pbf";

fn main() {
    let now = SystemTime::now();

    let coordinate_input: Vec<(f64, f64)> = vec![(40.424725, -3.690438), (40.421096, -3.688578)];
    // let coordinate_input: Vec<(f64, f64)> = vec![(40.45968221853669, -3.696199357287952), (40.41414441498608, -3.7210641923480385), (40.41748837725632, -3.6719311064057276), (40.464957584815146, -3.6815194281720793), (40.40582981421286, -3.7274261279407286), (40.43689338574444, -3.6507282923199917), (40.4195957152874, -3.7409059894230228), (40.44580387100087, -3.6410738165867174), (40.40986247774889, -3.6983632687377326), (40.39453474138612, -3.6740550402383167), (40.455966998307645, -3.6496863664378267), (40.43898900442006, -3.659894721208898), (40.44908612413164, -3.731184966397128), (40.46402659631937, -3.655089966474103), (40.435639176327406, -3.7367950138567076), (40.421897080457654, -3.6871333267945663), (40.40378327334573, -3.6438820455933376), (40.46401741737514, -3.7150227949612056), (40.39853262919868, -3.6577264756011374), (40.39882850451575, -3.6776176337348327), (40.41803447300695, -3.6882585686297524), (40.43481314699192, -3.670016640393406), (40.45615652360036, -3.6584796379112983), (40.4593370930648, -3.6579561924207447), (40.449809469112274, -3.640348957904225), (40.41330343362748, -3.6931788522782267), (40.41291393645797, -3.647084705697448), (40.40485313370108, -3.6914175976453714), (40.444462979819605, -3.7340319033313407), (40.4091494980056, -3.6547383445595605), (40.40855600406774, -3.649423227309151), (40.41662034790411, -3.653235833641703), (40.45226041795775, -3.631671020493101), (40.39936544346808, -3.675634293716435), (40.456018179271105, -3.6809657704255594), (40.39913455177876, -3.6827474631294153), (40.45109234974584, -3.6987227313299136), (40.46506236295274, -3.673778874554426), (40.44163738997094, -3.7131028534203727), (40.44426360773135, -3.6581561214168947), (40.418119736836886, -3.6296703944800734), (40.4328087282204, -3.7147894163896398), (40.42833807841645, -3.7292975543381384), (40.42309682591174, -3.63244252452793), (40.425394883967925, -3.700388159454904), (40.44253436742414, -3.7222537060431895), (40.44788492602304, -3.632110107340204), (40.41816326818757, -3.6850686278044154), (40.428648954634674, -3.664482196710723), (40.46552177062248, -3.671528816492505), (40.401927642579864, -3.685098063711926), (40.40564900453769, -3.691915621079472), (40.39798999397231, -3.6718386415449396), (40.44678241675134, -3.7263256236055615), (40.43825382437483, -3.702462456160235), (40.41810705737297, -3.690283691260803), (40.43067707363218, -3.7262837153087647), (40.44325741251985, -3.7349321131989055), (40.42768018195866, -3.7022652692423654), (40.44849963806865, -3.6733794939159963), (40.40359275992668, -3.685745708232124), (40.463576980718294, -3.695178335834395), (40.3978842456822, -3.6845258456690555), (40.44212508316398, -3.661871364377547), (40.444388745458895, -3.637190008385731), (40.442112651005495, -3.7241752300153754), (40.424770741903735, -3.706924532394265), (40.414365754280205, -3.6935376955996864), (40.43685777108082, -3.6471665714101067), (40.43212581333161, -3.6837280852375094), (40.43698224373448, -3.629554336504699), (40.422623780061855, -3.6703607327938466), (40.44230953834334, -3.686195769479123), (40.40653498749486, -3.7009021389915975), (40.410893304236886, -3.6417050475164885), (40.46194590943414, -3.655242135444336), (40.45101821653075, -3.688397776090211), (40.42779277786761, -3.697160874877661), (40.41176066281467, -3.703582801548902), (40.471887433630506, -3.693982937025113), (40.400198827238576, -3.723279773924908), (40.421982375539194, -3.696374349408854), (40.42762799063753, -3.7124850810611805), (40.44986526612503, -3.673407272204019), (40.42967022799188, -3.664681298755343), (40.41632371285828, -3.70245708775894), (40.44664761557899, -3.689615108248312), (40.44113579321836, -3.6290226754512527), (40.42138496404747, -3.65483770174064), (40.44029068553264, -3.728764764408976), (40.39813768237565, -3.6869443621786395), (40.448207214207116, -3.671363282772174), (40.42529371546785, -3.67020306614401), (40.46571157848504, -3.7104173666016145), (40.45879234011731, -3.6667799854046765), (40.46125169678382, -3.645701861573323), (40.44026603241081, -3.636218376727096), (40.46407909748253, -3.701376737795497), (40.44971214627771, -3.661752242208292), (40.431391670440775, -3.6835431927861175), (40.423388927667204, -3.7078717508457286), (40.3963493438624, -3.691909170066872), (40.46609155545836, -3.6566817696738347), (40.42874589209763, -3.683108840717926), (40.442939603268215, -3.732793346605521), (40.40564516087126, -3.7046926259085695), (40.42880836033047, -3.6761817470880556), (40.434451767591, -3.684283111662363)
                                                 // , (40.44898181087805, -3.727694719289203), (40.41478314396233, -3.666541669191066), (40.44406444270334, -3.6291400194394656), (40.4458506316909, -3.645380449877484), (40.413126497776176, -3.7047520111622214), (40.427033435337925, -3.668182616906757), (40.42302174939693, -3.648259076739721), (40.43407801520762, -3.6305954992228147), (40.417221917456416, -3.693913565387896), (40.427558820149564, -3.648746618069023), (40.42507232482549, -3.6494755379621373), (40.45171819543377, -3.6618274033055465), (40.44144414511743, -3.734324449186922), (40.43863633176511, -3.6437256927780943), (40.44035461581374, -3.7334092985443785), (40.47074587603393, -3.6826411140728004), (40.46064626691855, -3.672259804407521), (40.46226425988383, -3.67152421054351), (40.41184859610966, -3.6500881006982273), (40.422751887422045, -3.717825817326359), (40.42291898859062, -3.634785112563076), (40.431638206119864, -3.652666595623977), (40.44040765301692, -3.6774171689135726), (40.40725927552216, -3.6473621332223316), (40.41846531535522, -3.695635192438668), (40.43059400302558, -3.6275707102147017), (40.43064936036032, -3.7293378720972257), (40.43295068545107, -3.6471170889559885), (40.40856305684897, -3.6891519442342604), (40.4474718182782, -3.647196642824431), (40.44833188162688, -3.730503866698904), (40.436934965737926, -3.7362519180872793), (40.46454724151105, -3.667126717507305), (40.397940958094004, -3.658547478604741), (40.4467596919904, -3.6684159121555764), (40.40000964642222, -3.69909251835), (40.40007335475121, -3.667045530360389), (40.40859064216998, -3.720629796933635), (40.39941794152366, -3.658876098758592), (40.4586418703305, -3.6968954071557296), (40.405628570495374, -3.697337670143737), (40.44283733719773, -3.707015028239945), (40.41066945573042, -3.7184586066662964), (40.40841318587688, -3.6780430329888842), (40.42782201489406, -3.734384533893742), (40.41464988954448, -3.7179574548520153), (40.41333569579643, -3.6935561089317135), (40.40109582920952, -3.652803828076674), (40.41813822624924, -3.7124988492194024), (40.41180802640746, -3.6364287335396903), (40.39855400063633, -3.6602844061280546), (40.42184613250933, -3.6907070241980833), (40.414892172085914, -3.637050843032364), (40.41047110057697, -3.688810629893252), (40.41500800680723, -3.7383741585321455), (40.45477089560211, -3.6801551544587747), (40.41899304034691, -3.6756767707092184), (40.46476132292151, -3.6821817170925994), (40.42872548233026, -3.692522730112474), (40.42172311357957, -3.713395176626663), (40.4490225750535, -3.655524321827899), (40.41364722729378, -3.7093545407207533), (40.45441975044953, -3.670407297767151), (40.4560580740939, -3.719763984860406), (40.40273588182577, -3.6857493353356396), (40.40244161244345, -3.727524668510164), (40.46505388355216, -3.6596878092949674), (40.43648298525589, -3.635414028106944), (40.44962354942684, -3.6729604600400534), (40.43264225180207, -3.69286105554074), (40.42461200152459, -3.7284218546641457), (40.44258320663004, -3.671020382097829), (40.45398793322741, -3.7156550790233407), (40.44903568708954, -3.657508314523588), (40.42057130269291, -3.7202157658466253), (40.41708352255714, -3.648272860971599), (40.43442256616076, -3.6638617250385592), (40.400832394857694, -3.680929106474953), (40.435880018263234, -3.663308811673239), (40.4132120432198, -3.6655031452188074), (40.462307747393346, -3.7132410201283736), (40.46497962718742, -3.6443033408671233), (40.450279555179705, -3.6296452878425187), (40.45030559560985, -3.7029138130729575), (40.44846683005612, -3.658784937648374), (40.438390544787744, -3.644687805480594), (40.44517353719181, -3.693549450122309), (40.45882715637649, -3.6516696562800033), (40.42596476205829, -3.7340646522201775), (40.39628484994922, -3.6827816700456397), (40.419290351677084, -3.7292644665872254), (40.47234316145508, -3.678512898134455), (40.46511808386721, -3.706524102915091), (40.434811200507596, -3.6349349432716704), (40.43972469328815, -3.6480211725019935), (40.42379036351974, -3.73833155105756), (40.42695680313341, -3.7276496774641226), (40.46426978654805, -3.6962818547132774), (40.4132492621448, -3.680371953135738), (40.45791385451264, -3.721840382267865)
                                                 // , (40.45549297109219, -3.6569188514768975), (40.40949071124618, -3.6948945744467503), (40.436315734448925, -3.731879909917357), (40.412399878467866, -3.6652202763994888), (40.40839600817484, -3.7333989460264254), (40.454769994886554, -3.698165989497361), (40.4355052648175, -3.7118611023036854), (40.45690118004581, -3.6703854736430412), (40.40974325419643, -3.731790616954573), (40.414040693964495, -3.711708581926029), (40.406633946911704, -3.6770326960241513), (40.41961508723163, -3.7292598879294867), (40.45717031786477, -3.6730785707102713), (40.46689251532851, -3.6501482152473885), (40.44659152912064, -3.6275675140070676), (40.41077657318313, -3.644791690879782), (40.46151788668138, -3.690963282078495), (40.467075081622845, -3.6706074688246253), (40.41468259176233, -3.7385382674067826), (40.43241971955936, -3.659036556489671), (40.41425921042765, -3.6542309703733875), (40.40518768285577, -3.7143087747905037), (40.452365168163546, -3.649730567337449), (40.4284724361165, -3.7066913286835477), (40.427389128737104, -3.721326135071387), (40.45990138567298, -3.7169002999987115), (40.431314748351085, -3.6645425574919335), (40.41573090962055, -3.6735692034993477), (40.446138227845466, -3.7094281331817407), (40.4068738129653, -3.661562003618414), (40.40403828726165, -3.653683382471848), (40.46509468348498, -3.655134761080401), (40.46208449910407, -3.665124970940564), (40.4083948435113, -3.661515418302733), (40.4556942882692, -3.6626610021762596), (40.39733885612648, -3.6804544476521377), (40.445771809346354, -3.7215712921165482), (40.4317049640571, -3.7358526409725727), (40.42305850663656, -3.687819508197961), (40.41878572992456, -3.6941787695166117), (40.42761986929566, -3.699012005896114), (40.46762754638948, -3.687644907391631), (40.42816576279468, -3.7151568177556133), (40.414800481177835, -3.6368694344982857), (40.42784498088958, -3.681904096362597), (40.41992850842899, -3.702223146649796), (40.44210283412892, -3.7120346891722424), (40.40432547421421, -3.7260678182137927), (40.46886390155314, -3.6933495978339), (40.45744631014501, -3.6484021280915986), (40.46454075692838, -3.6843998991843767), (40.41598396867767, -3.728752332133719), (40.43833246310762, -3.7340013506402348), (40.44364436249871, -3.6897388718109263), (40.421274863390344, -3.724382140911768), (40.45774013725627, -3.6755819167007253), (40.420947932772634, -3.6549057099436677), (40.46103349362759, -3.7032580208811807), (40.463151466184875, -3.6419149082769993), (40.47296224196929, -3.6669135119149248), (40.409609058570545, -3.7008730182097014), (40.42513847732568, -3.6840338538415103), (40.441583022847446, -3.66063768399009), (40.43539254926079, -3.677547589940142), (40.46039315951995, -3.668154478435267), (40.45142074935556, -3.7296893933543886), (40.427147125181634, -3.628683585842027), (40.39988127524818, -3.6884030499532443), (40.41798880638181, -3.65008672633367), (40.45206063291473, -3.717329653020958), (40.4367992394274, -3.6403549511767794), (40.40637133047356, -3.702234586083823), (40.43213471131282, -3.6335696292076847), (40.45563320646395, -3.6796439808882737), (40.40412043872491, -3.6806684136041907), (40.39957491340224, -3.72181512875923), (40.44308842622529, -3.6743362963173056), (40.402814852662374, -3.7183112559544713), (40.42729150347177, -3.658008829989205), (40.422430244788565, -3.662379089750768), (40.44977797114197, -3.7047419992775663), (40.43418798360837, -3.651544182518738), (40.41706710503278, -3.7283300126534864), (40.44892116144094, -3.71683128673024), (40.405787090572225, -3.726289031729916), (40.415766807958285, -3.7318184605850337)
                                                 // , (40.4699873543758, -3.6787304556731053), (40.42593054080211, -3.717117232876183), (40.39798072771818, -3.6839120282056848), (40.43745727785748, -3.733497398137909), (40.411532182618025, -3.6770133079358542), (40.405921547516634, -3.6651974093950854), (40.459121074898825, -3.6486470176164), (40.42562768409705, -3.6934326806906355), (40.42406344872409, -3.6275514032233573), (40.410822527301974, -3.6404553090633454), (40.400627158601, -3.6800526928450723), (40.39926762696298, -3.683778838758395), (40.448258822226414, -3.7232779725117684), (40.424363595286444, -3.6900574158463884), (40.44577386644571, -3.7324980557927905), (40.44144493264214, -3.6412153699983145), (40.4161070609224, -3.718632035974198), (40.46278886684739, -3.715451875514012), (40.4636471803486, -3.678524634332154), (40.4637172679212, -3.654089381437655), (40.423732566455286, -3.6442244009078926), (40.46954171837031, -3.6805047238771285), (40.406346767872314, -3.709571336058396), (40.41760969479751, -3.6929977142737416), (40.42456338093415, -3.6600981586731876), (40.42359220979449, -3.734295771746403), (40.40519096088724, -3.6438400917271636), (40.413551207553695, -3.6749480142028244), (40.42340475220733, -3.6859405415234576), (40.43376858617393, -3.6730138115123094), (40.397525103662005, -3.6666957894899337), (40.453409430919926, -3.711383842356102), (40.41004275317694, -3.6723123034630847), (40.39859817612963, -3.6589267358367223), (40.4379969144344, -3.660781129717217), (40.42790983868693, -3.6887797971455214), (40.462898121494796, -3.6861171446832453), (40.442361330688435, -3.62712503456611), (40.402724121756606, -3.7090080546250004), (40.4434991216293, -3.701217364639965), (40.414557411427346, -3.657057583810376), (40.43815159169025, -3.6744539948752943), (40.4282499168852, -3.664343302882393), (40.46391348938551, -3.661487945114629), (40.428949786485525, -3.6856726392775006), (40.46715406289871, -3.709469415380358), (40.412655561700866, -3.6564773957048913), (40.44863854495479, -3.630618071369897), (40.40797802023627, -3.701028124073961), (40.4413794848233, -3.654946643744412), (40.422570037711324, -3.695311997915423), (40.40051740377158, -3.705344143032775), (40.41481451758415, -3.644630722424356), (40.42607014161918, -3.659570880898736), (40.41559057192731, -3.650701533177269), (40.42978393836319, -3.6825495680636315), (40.41509241221892, -3.644895752892068), (40.449073847096464, -3.652527330709055), (40.425657712996816, -3.6890093371230086), (40.40513147746601, -3.6814238529336842), (40.42727490925364, -3.6846843885513825), (40.39970543123726, -3.6538678461984055), (40.461271997943214, -3.6518437229924627), (40.46290366353241, -3.685165221212563), (40.44239830026924, -3.7214213474739073), (40.446665390936346, -3.632238226783902), (40.426888657444294, -3.6500089736440624), (40.43978026580786, -3.6542556375578794), (40.40143686646895, -3.6496272143640045), (40.41502970459081, -3.6945428419975754), (40.44011419904828, -3.7149787117497124), (40.44736556446711, -3.6659485544090766), (40.467913754682755, -3.6680007562781163), (40.438467690752, -3.634527129593234), (40.41050459731562, -3.705396710378587), (40.432199860100994, -3.6850825384083215), (40.460427655754174, -3.6711622287316916), (40.45937469577368, -3.6504124470252077), (40.40500031368882, -3.675769379876032), (40.461628457367915, -3.6716659124959174), (40.41875172763535, -3.7116495055579666), (40.450002658645914, -3.6795773255956115), (40.43750841177382, -3.6693963495753614), (40.42555885201086, -3.636581231888563), (40.43286839507827, -3.722709562236428), (40.42497189833987, -3.7337721740170386), (40.40095214218246, -3.696708544801047), (40.45663531911837, -3.6369288940781073), (40.452517474658315, -3.6626979064881686), (40.45371959315347, -3.699154300060142), (40.455326922413676, -3.6394042689707775), (40.40707519135597, -3.718822732362735), (40.44723534718675, -3.681902625100531), (40.43548526552402, -3.631516276185496), (40.394994454689666, -3.6719369440003655), (40.44338163089015, -3.711764305500868), (40.4538875963147, -3.693926305272647), (40.41048649728693, -3.6669599659079517), (40.41174805591306, -3.6585469512473603), (40.42825600933299, -3.7012144443410033), (40.40214746109682, -3.6861882992683515), (40.421060326322426, -3.6807279694965804), (40.454477918923565, -3.670735795944141), (40.448959774373535, -3.6560018763395212), (40.439450026646725, -3.6724140202126963), (40.3959847767095, -3.680470115269061), (40.46638560643119, -3.7088724234879966), (40.46189338453029, -3.6463478296292755), (40.4103498015682, -3.6439249923564017), (40.42093311467324, -3.723484143587835), (40.431865074811284, -3.713518363292722), (40.46343053292988, -3.694199595922702), (40.442817737198865, -3.665335118356554), (40.45714396551769, -3.6796740893638997), (40.41149599938187, -3.6516877729458592), (40.41555317628817, -3.64314348959345), (40.456583387751074, -3.708768877307961), (40.4016688381874, -3.7021088841837355), (40.44732962507189, -3.6984341989399034), (40.424913228403945, -3.689996851027033), (40.455755030830346, -3.6625173486297515), (40.40338283468787, -3.694951680297399), (40.440030372912254, -3.7243862397216465), (40.46976262903045, -3.6725651726858906), (40.44185700848748, -3.729254151538359), (40.42957744949881, -3.7272234245641918), (40.42303863863507, -3.663218924514063), (40.44944209495042, -3.7241925244371514), (40.44379796440051, -3.731809021178125), (40.44408612033566, -3.7217462659208826), (40.42795329281909, -3.6300903013412382), (40.444106627220506, -3.692107476735034), (40.45193029162054, -3.6839212948175053), (40.42371198462326, -3.6590432923238945), (40.436797540021175, -3.7335134333837674), (40.4278026873538, -3.702210435884924), (40.423534173089465, -3.6931177015923766), (40.44391825261851, -3.681632693913156), (40.47015294005262, -3.671758945969601), (40.46030391276407, -3.6678217857128375), (40.44168589821098, -3.6575043657934283), (40.40158834553803, -3.684341093478647), (40.42550895727437, -3.6920266495390393), (40.45351720651684, -3.660235303688606), (40.454339266860686, -3.6645345843048123), (40.41869861533847, -3.6789269114696985), (40.46726777250684, -3.6524688562658554), (40.41421303356259, -3.688949438704464), (40.43015620261586, -3.640097466068203), (40.43866525546291, -3.697301594372108), (40.40571349407029, -3.665510347405972), (40.41879559657143, -3.6695204663800016), (40.423838794682986, -3.645825079547767), (40.440165448137364, -3.7194912173034296), (40.44752444838869, -3.632416727909091), (40.43366369232003, -3.637723985024448), (40.444261433653395, -3.7300458795333933), (40.39749504726916, -3.6572182182968622), (40.46179076856894, -3.646381863399955), (40.41275793651811, -3.654659148810247), (40.41852983416169, -3.739970398758157), (40.45176025102327, -3.633038543727274), (40.40043229305662, -3.669563848753333), (40.423473542380506, -3.6392621717601714), (40.42407290299845, -3.7283249404305545), (40.439445749517766, -3.6641123561467723), (40.40316440235605, -3.722007802286038), (40.4350550395059, -3.7118150033753574), (40.41676375037986, -3.67621672173175), (40.422575330097565, -3.722149859392168), (40.406512279588405, -3.670511127643288), (40.4475610886707, -3.654825407705018), (40.44391234869516, -3.700901929921678), (40.40242973300101, -3.6535224400471358), (40.45848675902761, -3.666678721941921), (40.445593874314135, -3.7308254557033873), (40.45698145323249, -3.7088785920680363), (40.40683281673062, -3.6733794473052117), (40.40905406772886, -3.657584103973813), (40.4034188008026, -3.698962490627399), (40.42058907060774, -3.642140201704387), (40.45913949562389, -3.6980191626904744), (40.45577759981013, -3.7234372730086673), (40.423641076386424, -3.73953856433068), (40.444915776753, -3.6927373274140107), (40.45845603020832, -3.6798238071796145), (40.43275070445947, -3.6795749308828896), (40.40726670234882, -3.6932691774111235), (40.43435445136283, -3.628545134942123), (40.45463284804329, -3.642116437195078), (40.44376548176343, -3.708439915198476), (40.42429658275694, -3.6712120423109296), (40.41933131484686, -3.7079867106469697), (40.40863812136032, -3.643242098516591), (40.42271707338575, -3.6842787678475175), (40.399831337591515, -3.723887473011066), (40.455741610158405, -3.718284821899605), (40.43216165957128, -3.727151086397788), (40.45291906449721, -3.633007159959494), (40.414525832227724, -3.677247342955085), (40.42019606249964, -3.652537164684), (40.41644119509941, -3.702100783052267), (40.44781955326243, -3.6953524750885762), (40.44189689378208, -3.727160255778065), (40.44032654924862, -3.6450691400189092), (40.414038638888854, -3.6422639329561712)
    // ];
    println!("Generating a matrix for {} locations", coordinate_input.len());

    println!("Parse map file started.");
    let file_name: String = String::from(MAP_FILE_PATH);

    let map = Map::from(&file_name);
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    let mut graph: Graph<(), f64, Directed> = Graph::new();
    let mut nodes_on_graph = BTreeMap::new();
    let mut index_on_graph = BTreeMap::new();
    let mut coordinates: Vec<Coordinate> = Vec::new();

    println!("Reverse geocoding coordinates...");
    for coordinate in coordinate_input {
        let mut coord = Coordinate::from(coordinate);
        let node = map.reverse_geocode_node(coord);
        coord.node = Some(node);
        coordinates.push(coord);
    }
    println!("Reverse geocoding complete.");
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    for node in map.nodes {
        let graph_node = graph.add_node(());
        nodes_on_graph.insert(node.id, graph_node);
        index_on_graph.insert(graph_node, node);
    }

    for edge in map.edges.iter().filter(|&e| {
        e.properties.car_forward >= 1
    }) {
        let start = nodes_on_graph.get(&edge.source).unwrap().clone();
        let end = nodes_on_graph.get(&edge.target).unwrap().clone();
        let weight = edge.length();
        graph.add_edge(start, end, weight);
    }
    println!("Nodes on graph, {}", graph.node_count());
    println!("Edges on graph, {}", graph.edge_count());
    println!(" ✓ duration: {}s\n", now.elapsed().unwrap().as_secs());

    let mut distances: Vec<Vec<f64>> = Vec::new();
    let mut durations: Vec<Vec<f64>> = Vec::new();

    println!("Generating matrix");
    &coordinates.iter().for_each(|origin_coordinate| {
        let start_node = nodes_on_graph.get(&origin_coordinate.node.unwrap().id).unwrap().clone();
        let (distance, duration) = find_distances(start_node, (*coordinates).to_owned(), nodes_on_graph.clone(), graph.clone());

        distances.push(distance);
        durations.push(duration);
    });

    println!("Distances: {:?}\n", distances);
    println!("Durations: {:?}\n", durations);
    println!(" ✓ Total duration: {}s\n", now.elapsed().unwrap().as_secs());
}

fn find_distances(
    origin_node: NodeIndex,
    coordinates: Vec<Coordinate>,
    nodes_on_graph: BTreeMap<i64, NodeIndex>,
    graph: Graph<(), f64, Directed>,
) -> (Vec<f64>, Vec<f64>) {
    let mut distance: Vec<f64> = Vec::new();
    let mut duration: Vec<f64> = Vec::new();

    let res = algorithms::astar_multiple_goals(
        &graph,
        origin_node,
        |finish| coordinates.iter().map(|coordinate| nodes_on_graph.get(&coordinate.node.unwrap().id)).any(|node_index| node_index == Some(&finish)),
        |e| *e.weight(),
        |_| 0.
    );

    // let res = algorithms::dijkstra(
    //     &graph,
    //     origin_node,
    //     |finish| coordinates.iter().map(|coordinate| nodes_on_graph.get(&coordinate.node.unwrap().id)).any(|node_index| node_index == Some(&finish)),
    //     |e| *e.weight()
    // );
    // for (node_id, result) in &res {
    //     let cost = result.unwrap();
    //     println!("Found cost: {:?}", cost);
    // }
    // println!("Found costs: {:?}", res.len());
    const AVG_VEHICLE_SPEED: f64 = 25.00;
    for (_node_id, cost) in &res {
        match cost {
            Some(cost) => {
                distance.push(*cost);
                duration.push(f64::trunc(cost / AVG_VEHICLE_SPEED));
            }
            None => {
                distance.push(0.);
                duration.push(0.);
            }
        }
    }
    return (distance, duration)
}