#!/usr/bin/env julia

using MLBasedESC
using LinearAlgebra
using RobotOS
@rosimport sensor_msgs.msg: JointState
@rosimport std_msgs.msg: Float64
rostypegen()
using .sensor_msgs.msg, .std_msgs.msg

const USE_J2 = false
const PRECISION = Float64
const TRAINED_PARAMS = PRECISION[52.07558449351008, 2.0201087989665525, 37.29188156734188, -4.993719457686793, 0.34927954304939623, -0.08961827297661142, -0.9048831503448466, 0.20123804420075103, 0.18151466926980347, 0.547427843686472, 0.20160359709003245, -0.1731028561748276, -14.883699850978703, -3.186462993341891, 0.30087742046983834, -0.18828904408149363, -2.9491283661595866, -0.13283124107516173, 0.0018290709612463997, -0.006744496124000017, 0.01589595960309799, -0.3765880511582873, -0.008898919236145665, -0.1218398522522025, 0.12902272908586687, -0.00920615761436031, 0.10686997368928267, -0.9296072390427336, -0.009515767604165344, 0.01593269973201551, 0.029780613795172172, -6.4970215728649725, -0.053652072362441065, 0.06290082519219292, 0.13969600656518788, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.49285952733765453, 1.7622207872904176, 0.08959529548925124, 0.03263030523277364, -0.9340167400502981, -0.0783772288874565, -0.3930233629652298, 0.45725015263559693, 0.5114451133020311, -0.013350583327466452, -1.5224023067386951, 0.20735569456157363, 1.0597717408346696, 1.2206118594509607, 0.6412005451802895, 0.8086437921211861, -1.5932090993872863, 0.7168532114282931, 0.300245098369413, 0.16026150986302048, 0.6061947957837792, 1.258948417940038, 1.757627380532822, 0.5222960183289949, -0.6353758111183789, 0.4615966960070562, -1.0641947717672058, 0.14282099989482572, 0.4027574199133957, -0.3321526024695193, 0.36302210753155806, 0.871467322689247, -0.18336012061392212, 0.19335443121991422, -0.9550896404405955, 0.9893274409848611, -0.7557696186363969, 8.77968539243187, 0.216515872826157, 0.4212337956721467, 0.05837716552547394, 0.29675093577689826, 0.16204954292819512, -0.28872563415512154, -0.2802602408895925, -1.2339126187748435, 0.944414744043992, 0.22887704137462325, -0.10982819431697378, 0.35345598014603036, 0.05832101348976845, 0.08351499946689629, -0.2274794037679477, -0.10591643579768281, -0.05276603501436253, 0.1760117193417925, -0.1272941204857089, 0.13008612006872353, -0.16809194098895178, 0.2943184811834871, -0.18234129365920118, 0.3084237236413816, 0.017807680852648083, -0.1515512115517798, -0.23802040427541515, 0.14160234459613333, 0.1358576698349096, -0.11437926286266312, 0.16874185077338077, -0.06926193847809599, 0.21693018259121138, 0.15728832254332178, 0.02498646640660028, 0.07625128931258242, -0.08963349663573106, 0.1668175810471262, 0.30964683848215113, -0.31809149754141597, 0.23728063728471974, 0.21517454929062507, 0.3774882065991444, -0.18012532545860283, 0.1402687497484933, 0.2925590812622663, -0.3889866552076094, -2.1511300237230744, 0.003927135650526186, 0.33139433101745835, -0.06116425722349079, 0.22409224673369124, 0.36801154889419624, -0.23271687516106052, 0.00033936036308620214, -0.09112030871447974, -0.10148455452785307, -0.20421038464731575, 0.0676937995562539, 1.3011991923513282, 0.46240422273390963, 0.2811851903011469, 0.21897117689045142, -0.4494159324150805, -0.755053079297994, -0.38763185589366295, 0.3360291439227616, -0.500532377786618, -1.3759714116872241, -0.16773301491075468, 0.8989521352185891, 0.721772768901371, -0.17653508163238008, 0.42672416869481017, -0.7893714782394425, -0.296723306142072, -0.390898275848181, -0.17306520543959023, 0.18258544229927093, -1.1327585685196946, 0.7600050178711903, -0.03139023510513331, -0.21951224231384034, -0.657029673453566, -0.9115374235459756, 0.06218219003447157, -0.4768074693141376, -1.1288130039720057, -0.6199860515186164, 0.3819482633431921, -0.225988207103962, 0.04622213645729406, -0.6447226136602545, 0.5344862253961921, 0.7625743342956881, 0.5258893264412325, 0.07025489012104186, -0.1444532059694321, -0.2408961063021225, -0.45035755762992313, -0.36910535919057735, 0.41331267402478006, 0.10795298338843823, -1.2717080243019692, 0.20236414503065506, 0.5576572370107152, -1.3121746126256901, 1.6280059118667762, -0.17408687683292876, -0.19667986227436304, -0.9457072117687157, 0.34159377433767785, -0.33753521970037087, -0.16267643637254275, 0.5646668802958231, -0.2254173958936813, -1.7532976650720864, 0.48493477575119354, 1.2327462494686288, 1.1900014893631903, -0.3559169061048718, 0.3987808333286657, -1.7647853865040593, 0.7060886745971767, 0.48889227659122875, -0.06002520253104596, 0.9699360090807666, 0.365155446155659, 1.68346599815553, 0.1639953791355417, -0.7850239785739439, 0.8730911237598499, -1.1986227054575573, 0.3448624990555425, -0.9807133496164006, -0.6039715148481518, 0.48939119403690584, 0.4884666973620887, 0.42979530297203367, 0.3429597458939275, -1.0039639205014532, 0.8746046254748583, -1.1182789033498344, 7.529662694275248, -0.041150514921026865, 0.24013458675529603, 0.3918780405993901, 0.35966334266326583, 0.4244223825661952, -0.5707350883952352, -0.1523685246662013, -1.047751315981792, 0.9360743174167555, -0.449848985075552, 0.26479252248595697, 0.09534063988543273, 0.1799541093975675, -0.15003257903291464, 0.12680064892693313, 0.00013923039801484136, 0.1716724423721727, 0.19431377409431996, 0.12386878311792507, 0.313507096066786, 0.01515825535321657, 0.2801641391551332, 0.2793748029284029, 0.0319179289748563, 0.2615209215006538, 0.0009289833070342934, 0.049575919731676056, 0.18107509551498735, 0.0014864612060903927, 0.15384016299146477, 0.07618124636387104, 0.16436039868390276, -0.010570798361229893, 0.25996236254021127, -0.003577874312559387, 0.13849016844313514, -0.6143475355561948, 0.13738826453373, 0.9528391484802704, -0.21394727925381393, 0.02974365589994957, -0.46162339960734977, -0.07415603606173626, 0.10505564050452731, -0.353108582524081, 0.09347549588612851, -0.029500029012001102, -2.507488011114687, 0.1373543105857791, -0.0017727149081685715, 0.007926229719910005, -0.13600539769305686, 0.1747621906251622, -0.11401606546885694, 0.2605946992385531, 0.009786600628470424, -0.21808375208557282, 0.3933392137183185, 0.05474105211431472, 0.24667857693065853, 0.16654295932951818, -0.13410407086643908, -0.19729285578598518, 0.26997892997369616, -0.18663484869733046, -0.033291572383956046, -0.1180403969514277, -0.4913939078611153, -0.017395429774640103, 0.29550605619510734, -0.11112183588722072, -0.28315938112326206, 0.2294606850682303, 0.09959395045011327, 0.07218947407794486, 0.2675929309314098, 0.06084355264296361, -0.14168395754029273, 0.12788034537516904, -0.020560332505315953, 0.09574220149387307, 0.10054656117274006, 0.09727718414906981, -0.012311411484124827, -0.1349038308328562, -0.31953257963066456, -0.889098179234101, -0.06827706472761615, -0.18791743155076773, -0.22130373423206287, 0.2962851629028052, -0.16228715977661157, 0.09636747163474302, 0.1576872523202994, -0.29290828287758636, -0.86191477922753, -0.07921431443902843, 0.18608646687236077, -0.05646712843630851, -0.13846285214133813, 0.010203042868951642, -0.06912125112105791, -0.12712603425369157, -0.22442324187951995, 0.04299002040785016, -0.557063937638746, -0.012387396983467797, 0.30297692209926974, 0.20725461833512168, -0.3105121687026463, -0.3915681180977959, 0.022447347722882854, 0.08643643641172824, 0.24422079104330782, 0.030115405509941545, 0.047941942404366875, -0.027081908018323313, 0.45390546958783234, 0.2411287118000362, 0.06100991365695787, -0.01779220258203668, 0.16664796837425247, 0.05817619848378583, 0.25780690778468124, 0.46758849345131764, 0.005370665060763104, -0.06553315181421236, -0.16841313579532252, 0.5069040368935286, 0.02142661989912739, -0.43309759079399035, 0.1824111932422575, 0.11128639837071694, 0.5471888534566358, -0.05042284677969902, 0.1570276542732338, 0.035334262618388554, 0.08442733052533496, 0.21620067775818796, 0.40182681635228823, 0.003249799055757287, -0.25983555390760205, -0.22683261210985678, -5.856440800039913, 0.1598401492492385, -0.0056025198291982945, 0.47522577566651264, -0.06993674679998932, 0.4006566163086279, -0.0365280682482372, 0.2168669661266963, -0.1688208688244995, 0.17718579162436815, 0.05154293183276773, 0.19357674180455323, 0.056645003722670194, -0.2818277864091066, 0.05050657643292471, -0.25911000607795615, -0.07150607169140312, -0.20338625378724604, 0.11788206974080204, 0.08412293901229512, -0.4284265793638581, -0.20876956583324785, 0.0827252512951708, 0.09139849804250391, 0.16932358413096504, -0.25793650375785443, 0.07274935259481297, -0.15033066124912392, -0.09250889282852073, 0.1327945759951811, 0.02731278355022417, 0.2142356268709777, -0.1238070180331307, 0.17024265198735897, 0.09169397253469995, 0.10118972127643944, 0.24879521257414544, -0.18845329097094063, 0.20860956475572015, -0.6445271635108639, -0.20922130051219945, 0.13770500274639158, 0.047948051304731565, -0.06155741447390263, 0.07279174648195133, -0.28056883233349045, 0.2637395961574788, -0.2517597731823087, -1.311497569160881, 0.02162586711176325, 0.3073373354010528, 0.20435019901873816, 0.13811633156134634, -0.1458389846625156, -0.15862821236947414, 0.2698224725031481, 0.2668088261131481, 0.03824642077188133, -0.1349188554569585, 0.7260475627203333, 1.0078107173370587, 0.22466148301584335, 0.11597902441831713, -0.07928550492977006, -0.12357160701443047, -0.331678457285181, 0.6126363551851157, 0.46652435801238207, -0.374918797648059, -1.0016342678876913, 0.06486971784410661, 0.7621839607969852, 0.818145652877766, -0.675193921153185, -0.0983426182398968, -1.2868999090314603, -0.20072746923261317, -0.3521765430623255, -0.25615100408326463, 0.052061822837254725, -0.2890953308328324, 0.7697004175687533, 0.15509682367938962, -0.912680945434421, 0.4790436562557941, -0.02901507765500778, -0.43492982347969084, 1.5274141798143261, -0.6276022562339914, 0.40295592260255003, 0.5850200809486974, -0.1388387162884825, 0.099701077067514, -1.158162258013588, -0.39847121687452913, 0.11762102263770728, 0.43605590502566705, 0.6836158399719797, 0.23372702858594682, -0.24206495747790807, 0.6515239395675513, -0.2933986867914331, -0.13362486780154403, 0.7780774776097367, -1.221803770603357, 0.024608853756568422, -0.19984873807413944, 0.41276869622145124, 1.9754289177488626, 0.22804003817852325, -0.6574733805658164, -1.0945246814214804, -0.2718800931967652, -0.4108529632741402, -0.22226213328746872, 0.1934690227910445, 0.16832646567943274, -1.2861000971925949, -0.1493094836552633, 1.709424398143157, 1.7936140168030261, 0.7567308365293887, 1.0183636713208208, -1.8790503240525205, 0.11072297735496385, 0.3808098193908817, 0.2798251857930826, 1.180392901327355, 1.5131486750371757, 1.7825537276566525, 0.4249669612005829, -1.274706399954738, -0.017242060571137103, -1.3426853259491078, 0.6405360319332881, 0.8842577662624614, -0.9304428541128228, -0.11222401906029093, 0.8953648730681921, -0.2960869589473444, 0.0468769169413298, -0.44024868012701807, 0.8527565980208869, -1.0921544419513245, 10.357918936539459, -0.11301836309211981, -0.5279536962173361, -0.1567606398033267, -0.16032037738213423, -0.12326202607245239, -0.7712208948410771, -0.10366142451489048, -1.5353553598692873, 0.9015240344601979, -0.41726287913278354, -1.276502751647269, 1.2132991834011815, -0.2999502910991376, -1.2755438023262262, -1.2510195123861685, 0.4102869597929611, -0.40100064059925317, 0.10142677561896292, 0.5736656911106857, 0.012002032203947922, -1.2611622178156743, 0.23287769529093966, 0.287628106449308, 0.6332347782732256, -0.08884083861743695, 0.21498780059273379, -1.573815527662548, -1.4783149958332262, -1.410603873025062, -0.4758034304954491, 0.4022193908332108, 0.46872528116746814, 1.1546173599045018, 0.4138876051257665, -0.6869910784576729, 0.7058676781716053, -1.0750098869884335, 0.41507594006146603, -0.702188520368303, -0.7083113986911951, 0.5164554017716155, 0.46703197541528413, 0.27592780070358697, 0.8090660753878763, -0.8753608087726199, 0.18075941409936552, -1.7668404920225251, 6.7209512201775174, 0.11977609748106507, 0.801044212698482, 0.32592069526417783, 0.3407645049477557, -1.2321779235975556, -0.3879910576484688, -0.3291179276708785, -1.1705226198605423, 0.6959675551599912, -0.1155911524123496, 0.12052825291846497, -0.06268851096666987, 0.04343377748875472, -0.42365986892445506, -0.33843389798771223, 0.2481693258900775, -0.10227459133865546, -0.19929055485818858, 0.22977571303661748, 0.037525407778232, -0.01685866455488709, -0.17003468689222404, 0.22588080932581922, 0.11106210654584718, 0.060491644007081576, -0.0710601602865278, -0.23555531386268586, 0.07993540967574615, 0.14803008310208324, 0.06803381849425798, -0.19612179025722284, 0.13459278940787822, -0.17499092003966524, -0.194305539613438, 0.031290312115788886, 0.047456070191489404, -0.10521143919591126, -0.22924219379763772, 0.04116840180356751, 0.1495641350893573, 0.07128986121720952, 0.24910443975849364, -0.03610398102810916, 0.32910953500942913, -0.2560557805329537, 0.17304637851313984, -0.020708127305391476, -1.8546454678958213, 0.24038210465107798, -0.13537586387183878, 0.16691012814549266, 0.31613096777272515, -0.030814124151974057, -0.3304987254081021, 0.15518856565883773, 0.0891116120763039, 0.16059008255193333, -0.23447746152342694, 0.8851192474067332, 0.5607624948757574, 0.816556324170071, 0.05076730004165815, 0.2742169054046091, -0.10513107131229785, 0.07821334502280956, 0.10343623690918508, 0.2607074111057668, 0.21201094390476574, -0.6844655569597501, 0.0667386114582603, 0.7140530101326413, 0.556276667031787, -0.20679848004023807, 0.058909588599178025, -0.7774279293707548, 0.13513314965341414, 0.29929208997147516, -0.31095067144015104, -0.3611531725163894, -0.2174650415806456, 0.5307662290316516, -0.21110808793819363, -0.33365092536686286, 0.4333066995074389, -0.19335474919405354, -0.5852277276408542, 3.3106028662618536, -0.49193299149384806, 0.3933069420311379, 0.12623374869757722, 0.10330705317426458, 0.2993525689073168, -0.07926305834849766, -0.034024540107271836, 0.20415512856228854, -0.17490260344795966, 0.11425145764384703, -0.14488269006376292, -0.2467524865994821, 0.2908789368189192, 0.07369714080826238, 0.08550327170143111, 1.7679956792808424, -0.6866484212777966, 0.3026562654646692, 0.0420345015547668, -0.5963996187440792, 1.4037492818940018, -0.06915022310189138, 0.16798403697830006, 0.2995000880095993, 0.2788237259966125, -0.20895506274022624, 0.3389419096673873, 0.2808059189253172, -0.14605494478838132, -1.0051306458271247, -0.04477605897864291, 1.094212474351912, 1.2936242252487855, -0.11234729614291772, 0.011912634897957599, -1.2737404771980712, 0.25947229035728026, 0.3203976935650463, -0.45539769965465093, -0.3502075882873611, -0.7005002583022215, 0.9810689539163324, 0.08175696209871666, -1.049658265259277, 1.2019934756433397, -0.5971588553579527, 0.3826044765879873, 0.9170271565172412, -0.36124809528845697, 0.5053041253569227, 0.39328608346661886, 0.2320818522075692, 0.13762481313920794, -0.9455801653052929, -0.3410609349433374, -0.25530982298084043, 1.3059250050338083, 0.42179956566196286, 0.5550747419503891, 0.21842967216632667, 0.8199386483159052, 0.9617407385070376, 0.24100134180083452, 0.36251738624830776, -0.9517072506117319, 0.44276613266787956, 0.24263214550059636, -0.03994234969119358, -0.22523040531255412, -0.21821609619055174, 0.037505850842705954, -0.09662136828916419, 0.020273068933360828, 0.11771555209783141, -0.05674263435143224, -0.089222684488314, 0.3795919114335599, 0.039751022421400016, -0.21404895047351027, 0.2463468998517374, -0.09102906072432976, 0.0957154137070972, 0.15722370924617815, 0.17428383078850143, -0.017988126452980705, -0.10386561320271874, -0.22897816749748712, 0.08185251614743759, 0.32696172263322415, -0.26099990117606264, 0.4984968614697972, 0.03962868778113374, 0.10222323347476532, -0.45964370136450033, -0.02104891367335727, -3.5310158339085995, -0.28989963972041793, 0.04711964845755214, 0.2418337214043654, 0.14939778252761557, 0.049449431853599374, -0.4872228955210272, 0.26414775764803894, 0.004524764884840363, 2.8049888399980984, -0.06395152717167263, -0.20484520188565175, 0.00839265390199002, -0.043999753454329736, -0.08252321935848639, -0.2887055126287607, -0.25853241370757246, -0.15768559346783975, 0.28801203593563934, -0.30195695959217955, -0.09680441621963398, -0.16054206168953045, -0.043284833565248516, 0.0462167815785277, 0.4252656047435811, -0.1405804951092347, -0.11496926369473283, 0.062466140809784784, -0.14672956103488877, -1.3846881184589206, 0.1862960714521791, 0.09189174302601147, 0.058127955832727514, 0.21641542208889636, 0.19547854566898895, 0.020870209215529567, 0.2926109936570343, -0.17193603038951344, -0.11171465925069891, 0.09293630929086931, -0.018886360761147778, -0.0017538808355898926, -0.04255045814572754, -0.18662529435423153, -0.20167132118299022, -0.1603745373913727, -0.00735064133626343, 0.056131542937400546, -5.648768224154585, -0.04135530010836329, 0.1255753685266268, 0.13283035500746632, -0.0890921510726319, 0.1855078950713014, -0.10246157391062678, -0.24701048121398617, 0.43060375803797174, 0.2258769456452381, -0.17421867903370533, 0.01163835862267677, -0.10544465302242671, -0.32526578570110776, 0.1317130072727887, 0.07992920078251807, 0.11299423088428551, 0.15354874427149315, 0.3825097168918239, -0.16245708082523763, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.021659254944515042, -0.5183789296782868, -0.18843942729493354, 1.006248686829501, 0.7271312691871091, 0.08614876952357549, 0.20959997621466456, 0.014866590649549425, 0.11798407382844768, 0.6250037840919529, 1.4256349000810729, 0.3562930146186068, -0.2489231280580737, -0.23458639011754248, -0.14733759967289686, -0.09309276763308386, -0.3807371717782782, 0.22449439921888506, 0.2492240460844804, -0.2243031938579772, -0.19134233309679757, -1.4792155422639743, -0.3384411492962878, 0.06682302760928895, -2.0734363475906226, 0.24790752337282101, 0.4593453860708097, -0.12408809917419644, 0.564985862409722, 0.6114871111819904, 0.03608797978641076, -0.07163921284228207, 0.30621819946438317, 0.06610035377164945, -0.989937064526781, -0.14368359040036355, 0.7157600561825381, -2.973642746241, 0.22898570068530297, 0.20266285912817017, 0.10480515454593864, 0.09075103386906615, 0.2518706942397028, 0.648233070325443, -0.1312358660212357, 0.5496371938942523, -0.10192038643506025, 0.7065768200457238, 0.0, 0.0, 0.0]
const I1 = PRECISION(0.0455)
const I2 = PRECISION(0.00425)
const m3 = PRECISION(0.183*9.81)
const b1 = PRECISION(0.001)
const b2 = PRECISION(0.002)

function create_true_hamiltonian()
    mass_inv = inv(diagm(vcat(I1, I2)))
    pe(q) = m3*(cos(q[1])-one(eltype(q)))
    Hamiltonian(mass_inv, pe)
end

function create_learning_hamiltonian()
    massd_inv = PSDMatrix(PRECISION,2,2)
    vd = NeuralNetwork(PRECISION, [2,16,48,1], symmetric=true, fout=x->x.^2, dfout=x->2x)
    Hamiltonian(massd_inv, vd)
end

function create_ida_pbc_problem()
    input = PRECISION[-1.0,1.0]
    input_annihilator = PRECISION[1.0 1.0]
    ham = create_true_hamiltonian()
    hamd = create_learning_hamiltonian()
    if USE_J2
        J2 = InterconnectionMatrix(
            SkewSymNeuralNetwork(PRECISION, 2, nin=4),
            SkewSymNeuralNetwork(PRECISION, 2, nin=4)
        )
        p = IDAPBCProblem(ham, hamd, input, input_annihilator, J2)
    else
        p = IDAPBCProblem(ham, hamd, input, input_annihilator)
    end
    p.init_params[:] = TRAINED_PARAMS
    return p
end

function update_state(msg::JointState, state::Vector)
    state[1] = msg.position[1]
    state[2] = msg.position[2]
    state[3] = msg.velocity[1]
    state[4] = msg.velocity[2]
end

function compute_control(x::Vector, swingup_controller::Function)
    q1, q2, q1dot, q2dot = x
    xbar = [sin(q1-pi), sin(q2), q1dot, q2dot]
    if (1+cos(q1)) < (1+cos(pi-pi/10)) && abs(q1dot) < 5.0
        K = [-7.409595362575457, -0.05000000000000429, -1.1791663255097424, -0.03665716263249201]
        effort = -dot(K,xbar)
        return clamp(effort, -2.0, 2.0)
    else
        I1 = 0.0455
        I2 = 0.00425
        M = diagm(vcat(I1, I2))
        # q1bar = rem2pi(q1-pi, RoundNearest)
        # q2bar = rem2pi(q2, RoundNearest)
        q1bar = q1-pi 
        q2bar = q2 
        qbar = [q1bar, q2bar]
        # effort = energy_shaping_controller(x)
        effort = swingup_controller(qbar, M*xbar[3:4])
        return clamp(effort, -0.85, 0.85)
    end
end

function energy_shaping_controller(x::Vector)
    q1, q2, q1dot, q2dot = x
    w1, w2, w3, w4 = (-0.09, 0.05, 0.0011375, -0.005)
    q1bar = q1-pi
    q2bar = q2
    effort = w1*q1dot + w2*cos(q1)*q1dot + w3*q1dot^3 + w4*q2dot
    return clamp(effort, -1.0, 1.0)
end

function main()
    init_node("ida_pbc_controller")
    state = zeros(Float64,4)
    pub = Publisher{Float64Msg}("theta2_controller/command", queue_size=1)
    sub = Subscriber{JointState}("/joint_states", update_state, (state,), queue_size=1)
    prob = create_ida_pbc_problem()
    θ = TRAINED_PARAMS
    u = controller(prob, θ, damping_gain=1e-3)
    # u = energy_shaping_controller
    loop_rate = Rate(500.0)
    while !is_shutdown()
        header = std_msgs.msg.Header()
        header.stamp = RobotOS.now()
        effort = compute_control(state, u)
        # effort = clamp(effort, -2.0, 2.0)
        gear_ratio = 1.0
        eta = 0.98
        k_tau = 0.230    # N-m/a
        current = effort / gear_ratio / k_tau / eta
        cmd = Float64Msg(current)
        publish(pub, cmd)
        rossleep(loop_rate)
    end
    safe_shutdown_hack()
end

function safe_shutdown_hack()
    run(`rostopic pub -1 /theta2_controller/command std_msgs/Float64 "data: 0. "`, wait=false);
end
Base.atexit(safe_shutdown_hack)

if !isinteractive()
    main()
end
