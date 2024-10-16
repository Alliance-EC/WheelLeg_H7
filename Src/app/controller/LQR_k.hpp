#pragma once
#include <cmath>
namespace app::controller {
// Function Definitions
//
// LQR_k
//     K = LQR_k(Ll,Lr)
//
// Arguments    : double Ll
//                double Lr
//                double K[40]
// Return Type  : void
//
void LQR_k(double Ll, double Lr, double K[40]) {
    double K_tmp;
    double b_K_tmp;
    double c_K_tmp;
    double d_K_tmp;
    double e_K_tmp;
    double f_K_tmp;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    //     This function was generated by the Symbolic Math Toolbox version 23.2.
    //     2024-10-11 21:17:38
    t2      = Ll * Ll;
    t3      = std::pow(Ll, 3.0);
    t5      = Lr * Lr;
    t6      = std::pow(Lr, 3.0);
    t4      = t2 * t2;
    t7      = t5 * t5;
    K_tmp   = Ll * t5;
    b_K_tmp = Ll * t6;
    c_K_tmp = Lr * t2;
    d_K_tmp = Lr * t3;
    e_K_tmp = t2 * t5;
    f_K_tmp = Ll * Lr;
    K[0]    = ((((((Ll * 93.8489491867109 - Lr * 149.41858263746241) - t2 * 244.04051117333171)
               + t3 * 243.66341503222469)
              - t4 * 63.441404481276322)
             + t5 * 586.54815168572532)
            + ((((t6 * -996.2689621202943 + t7 * 635.24031741791) - K_tmp * 167.47917566365359)
                + b_K_tmp * 344.49414789720709)
               + c_K_tmp * 629.85703289792775))
         + (((d_K_tmp * -468.83690736788031 - e_K_tmp * 261.70230594779372)
             - f_K_tmp * 162.0664179449908)
            - 7.333543302083485);
    K[1] = ((((((Ll * -153.51555855896359 + Lr * 96.347164925614308) + t2 * 582.0690736117466)
               - t3 * 975.65301329934164)
              + t4 * 625.71631135791074)
             - t5 * 284.07233660894332)
            + ((((t6 * 346.3513978236108 - t7 * 125.8936994243206) + K_tmp * 547.09080144724317)
                - b_K_tmp * 495.24156443283408)
               - c_K_tmp * 236.096806075552))
         + (((d_K_tmp * 337.82333822163957 - e_K_tmp * 130.98864213159331)
             - f_K_tmp * 106.8777879626769)
            - 7.2597954779516023);
    K[2] = ((((((Ll * -153.0720927351951 + Lr * 77.777088648783561) + t2 * 480.17582277894928)
               - t3 * 769.63469933466445)
              + t4 * 525.998480410417)
             - t5 * 827.99476809362977)
            + (((((t6 * 2188.260618061749 - t7 * 1849.683058585153) - K_tmp * 656.77498598828811)
                 + b_K_tmp * 84.266153460613737)
                - c_K_tmp * 665.98520595499861)
               + d_K_tmp * 244.04766984756441))
         + ((e_K_tmp * 826.1792422602457 + f_K_tmp * 416.59269884002128) + 25.41154043047144);
    K[3] = ((((((Ll * 71.5185310406178 - Lr * 143.3836221015) - t2 * 884.023999837692)
               + t3 * 2330.6745663656379)
              - t4 * 2029.568479067811)
             + t5 * 381.36053633162749)
            + ((((t6 * -516.06736219693028 + t7 * 277.36400772357308) - K_tmp * 935.01655127388517)
                + b_K_tmp * 557.63388704520889)
               - c_K_tmp * 733.346225546744))
         + (((d_K_tmp * 289.014939320175 + e_K_tmp * 693.84652218704093)
             + f_K_tmp * 550.77178192868394)
            + 25.217162431998108);
    K[4] = (((((Ll * 59.602863281367028 - Lr * 83.420139729532778) - t2 * 157.98666375769031)
              + t3 * 167.61584756298879)
             - t4 * 54.353642485569573)
            + (((((t5 * 320.20698906616752 - t6 * 561.935501321348) + t7 * 371.60118136751282)
                 - K_tmp * 18.545622974251451)
                + b_K_tmp * 139.3479237925462)
               + c_K_tmp * 403.29472449724051))
         + (((d_K_tmp * -290.27541385579389 - e_K_tmp * 185.64689418654191)
             - f_K_tmp * 131.18231471646379)
            - 5.8957277356249724);
    K[5] = (((((Ll * -87.192933579669187 + Lr * 62.4285311311567) + t2 * 319.49025579032582)
              - t3 * 550.11580319646134)
             + t4 * 364.32739789820141)
            + ((((t5 * -188.6007286423434 + t6 * 241.88318602377331) - t7 * 98.19878846462133)
                + K_tmp * 351.08287042516508)
               - b_K_tmp * 313.51689925131751))
         + ((((c_K_tmp * -69.482673151187029 + d_K_tmp * 141.5002663890663)
              - e_K_tmp * 97.7579393477801)
             - f_K_tmp * 93.4198427141099)
            - 5.8536139950677786);
    K[6] = (((((Ll * -94.727551368034469 + Lr * 42.7247691607787) + t2 * 296.1167911890816)
              - t3 * 470.75732728841109)
             + t4 * 319.82265980734559)
            + ((((t5 * -470.9745216259019 + t6 * 1285.59990081705) - t7 * 1104.042224459635)
                - K_tmp * 489.09994559227562)
               + b_K_tmp * 127.2545266520569))
         + ((((c_K_tmp * -421.253514634262 + d_K_tmp * 149.69168437280351)
              + e_K_tmp * 545.81997118452875)
             + f_K_tmp * 273.15746647500168)
            + 16.185942140881149);
    K[7] = ((((((Ll * 41.647225291382547 - Lr * 91.453998717539264) - t2 * 525.58163239847465)
               + t3 * 1410.5624717574269)
              - t4 * 1252.01205493878)
             + t5 * 237.66550026559719)
            + ((((t6 * -324.01292862885782 + t7 * 168.488210404723) - K_tmp * 599.21831951498621)
                + b_K_tmp * 380.2810795766307)
               - c_K_tmp * 550.58049875282381))
         + (((d_K_tmp * 296.13565556850318 + e_K_tmp * 418.58864948484148)
             + f_K_tmp * 372.94955930726968)
            + 16.062591603675859);
    K[8] = (((((Ll * -24.416282290692081 + Lr * 37.917031995437412) + t2 * 102.7346868600271)
              - t3 * 197.61334351082081)
             + t4 * 141.586046257338)
            + ((((t5 * -79.809289257981845 + t6 * 58.840100046699661) + t7 * 4.9793803748300043)
                + K_tmp * 106.18575302028709)
               - b_K_tmp * 81.925843913950075))
         + ((((c_K_tmp * 37.022916022807223 + d_K_tmp * 7.097546726072804)
              - e_K_tmp * 69.792447614896119)
             - f_K_tmp * 35.6258430435615)
            - 6.4344885350629246);
    K[9] = (((((Ll * -40.096355828311282 + Lr * 26.47970030853438) + t2 * 77.673319027656049)
              - t3 * 36.677416468246832)
             - t4 * 31.65382983998952)
            + ((((t5 * -94.009299777045584 + t6 * 159.52153600799261) - t7 * 110.8200039406731)
                + K_tmp * 36.4056909919268)
               - b_K_tmp * 40.532803760258943))
         + ((((c_K_tmp * -165.64948864744369 + d_K_tmp * 151.6216798031669)
              + e_K_tmp * 31.278811287258151)
             + f_K_tmp * 29.774490808429221)
            + 6.44104911702588);
    K[10] = (((((Ll * -7.1732476820906559 - Lr * 39.924933446598807) + t2 * 37.9340613074847)
               - t3 * 96.680679144150545)
              + t4 * 90.5203966425622)
             + ((((t5 * 219.67181019834629 - t6 * 423.54244814323681) + t7 * 301.0125279750485)
                 + K_tmp * 76.176973785387)
                - b_K_tmp * 2.7010983462650562))
          + ((((c_K_tmp * 219.723569986729 - d_K_tmp * 157.724111532262)
               - e_K_tmp * 77.615325779342939)
              - f_K_tmp * 119.6398515139627)
             - 5.7508725895494361);
    K[11] = (((((Ll * 48.2581295018468 - Lr * 1.5707819881093359) - t2 * 225.17503050484419)
               + t3 * 405.02240622535658)
              - t4 * 256.82255601420769)
             + ((((t5 * -24.725426621054879 + t6 * 90.445643783197639) - t7 * 77.086384010084927)
                 - K_tmp * 242.7820578850627)
                + b_K_tmp * 111.12702867820281))
          + ((((c_K_tmp * -33.593525583405643 - d_K_tmp * 112.93331247452871)
               + e_K_tmp * 186.01632807764511)
              + f_K_tmp * 114.3097611809428)
             + 5.7726650410600948);
    K[12] = (((((Ll * -7.5383508432967714 + Lr * 11.24038034305083) + t2 * 32.541089292614672)
               - t3 * 63.785903028020627)
              + t4 * 46.16582112893304)
             + ((((t5 * -18.405238150395931 + t6 * 5.1968916582628122) + t7 * 11.206574239816019)
                 + K_tmp * 37.634445759859943)
                - b_K_tmp * 27.76688336280008))
          + ((((c_K_tmp * 15.1213126127507 + d_K_tmp * 1.301782859892455)
               - e_K_tmp * 24.950847311163749)
              - f_K_tmp * 14.254362404847081)
             - 1.6655528000181781);
    K[13] = (((((Ll * -11.65702194630879 + Lr * 7.9807446368340988) + t2 * 17.764507233905249)
               + t3 * 2.559428037492359)
              - t4 * 21.838754088697431)
             + ((((t5 * -28.482565180279259 + t6 * 49.649287769082378) - t7 * 35.3027081650844)
                 + K_tmp * 10.49403692241771)
                - b_K_tmp * 11.68763228161953))
          + ((((c_K_tmp * -56.589990491131907 + d_K_tmp * 53.8030910618146)
               + e_K_tmp * 8.8974639545046088)
              + f_K_tmp * 10.695714920846219)
             + 1.664222982174375);
    K[14] = (((((Ll * -1.611085496479439 - Lr * 13.4823866865133) + t2 * 7.433671402257267)
               - t3 * 18.549920942536549)
              + t4 * 17.077673573732518)
             + ((((t5 * 68.276316230743092 - t6 * 125.6326156172215) + t7 * 86.008462959639132)
                 + K_tmp * 18.302109559442808)
                + b_K_tmp * 5.3580604711692352))
          + ((((c_K_tmp * 68.223298446705172 - d_K_tmp * 46.3730414494953)
               - e_K_tmp * 24.41352954665297)
              - f_K_tmp * 37.699177441529848)
             - 1.535017934401937);
    K[15] = (((((Ll * 15.85604831922166 - Lr * 0.89108545712515508) - t2 * 70.001813104240227)
               + t3 * 121.8740765735039)
              - t4 * 74.5955002415063)
             + ((((t5 * -6.0830116568246124 + t6 * 21.815900588189528) - t7 * 18.844907761188519)
                 - K_tmp * 72.740952433635627)
                + b_K_tmp * 35.78696256097345))
          + ((((c_K_tmp * -15.166091087742389 - d_K_tmp * 31.336166586795631)
               + e_K_tmp * 52.855232803382911)
              + f_K_tmp * 38.861542130436717)
             + 1.54145949913016);
    K[16] = (((((Ll * 51.592768786442811 - Lr * 117.6029281271165) - t2 * 210.40691707773621)
               + t3 * 368.10355322951477)
              - t4 * 239.27690665879851)
             + ((((t5 * 107.2838070809143 + t6 * 28.100435693538049) - t7 * 132.5031131015954)
                 - K_tmp * 392.0275445060222)
                + b_K_tmp * 296.726128322157))
          + ((((c_K_tmp * -7.3437793233102067 - d_K_tmp * 103.8888963250108)
               + e_K_tmp * 170.96165255941551)
              + f_K_tmp * 124.9768503053142)
             - 3.1360099538665369);
    K[17] = (((((Ll * -20.531063112729321 - Lr * 30.272793667507049) + t2 * 194.4575060161539)
               - t3 * 531.9097521643971)
              + t4 * 497.51635517301622)
             + (((((t5 * 138.84784533319609 - t6 * 329.83075685003928) + t7 * 294.50762265015288)
                  + K_tmp * 144.99038237473539)
                 - b_K_tmp * 86.764447640589879)
                + c_K_tmp * 475.77501774812163))
          + (((d_K_tmp * -417.02007616957837 - e_K_tmp * 124.51481609340109)
              - f_K_tmp * 186.89933240638541)
             - 2.814387421439521);
    K[18] = ((((((Ll * -62.918184006492218 + Lr * 175.48469589649571) + t2 * 221.87380750904759)
                - t3 * 387.6700114864197)
               + t4 * 292.10351835863111)
              - t5 * 733.43861724900307)
             + ((((t6 * 1413.9582522840431 - t7 * 975.47722872637189) - K_tmp * 124.974120843036)
                 - b_K_tmp * 235.88912852722339)
                - c_K_tmp * 450.36373937497132))
          + (((d_K_tmp * 171.9764104363579 + e_K_tmp * 412.15747070190668)
              + f_K_tmp * 257.21262194036188)
             + 6.9163640102755224);
    K[19] = (((((Ll * -5.6030637144085063 - Lr * 27.336457761722968) - t2 * 197.451649571294)
               + t3 * 804.91557395688085)
              - t4 * 917.0810136750473)
             + ((((t5 * 29.611600299942289 - t6 * 19.99938861117128) - t7 * 20.135373577825259)
                 + K_tmp * 156.8152562123137)
                - b_K_tmp * 67.273761770405159))
          + ((((c_K_tmp * -430.77959239292369 + d_K_tmp * 646.74923831607646)
               - e_K_tmp * 79.372662458366264)
              - f_K_tmp * 18.46776543141987)
             + 5.3268635568129836);
    K[20] = (((((Ll * 3.315347321554508 - Lr * 8.4637550291581132) - t2 * 14.2241996173621)
               + t3 * 24.47457852397191)
              - t4 * 14.74106451849268)
             + ((((t5 * -16.31150892014599 + t6 * 31.202600630894061) - t7 * 23.556730544269389)
                 - K_tmp * 31.0280910224205)
                + b_K_tmp * 9.7000955772314086))
          + ((((c_K_tmp * -11.3309604427884 - d_K_tmp * 5.640773359236813)
               + e_K_tmp * 26.858354714074011)
              + f_K_tmp * 16.439643595573258)
             - 0.64044540523620963);
    K[21] = (((((Ll * -2.4322006944796342 - Lr * 0.33333011135610657) + t2 * 21.960047169370839)
               - t3 * 61.863722713253551)
              + t4 * 60.44143780727741)
             + ((((t5 * 2.6167807931022229 - t6 * 9.9316504253583489) + t7 * 11.40707721548398)
                 + K_tmp * 16.161174771465468)
                - b_K_tmp * 9.6704484323791355))
          + ((((c_K_tmp * 95.745855041218348 - d_K_tmp * 83.726636694395637)
               - e_K_tmp * 9.8886916555013027)
              - f_K_tmp * 45.477523352380217)
             - 0.22385524954222491);
    K[22] = (((((Ll * -5.0799896072444941 + Lr * 6.9280478417925053) + t2 * 21.496262251581541)
               - t3 * 44.377122163820523)
              + t4 * 40.0976020950154)
             + ((((t5 * -13.0654416994839 + t6 * 22.49651026648031) + t7 * 1.963408879245683)
                 + K_tmp * 9.4315891900343551)
                - b_K_tmp * 56.834204416247943))
          + ((((c_K_tmp * 5.1651259871333561 - d_K_tmp * 33.570353405185777)
               + e_K_tmp * 50.076828970733473)
              - f_K_tmp * 0.392465839444588)
             + 1.8492633831574929);
    K[23] = (((((Ll * 5.23173569128668 + Lr * 2.9906802826336381) - t2 * 55.722380261652731)
               + t3 * 181.23103605204071)
              - t4 * 195.40133161820469)
             + ((((t5 * -12.39593116766828 + t6 * 20.580265022106939) - t7 * 21.691137936358249)
                 + K_tmp * 3.3353796601485151)
                + b_K_tmp * 12.53180665090861))
          + ((((c_K_tmp * -146.55920562809609 + d_K_tmp * 184.088085983737)
               - e_K_tmp * 16.7417616272728)
              + f_K_tmp * 30.3812250522981)
             - 0.1031627631992558);
    K[24] = (((((Ll * -26.93708534770963 - Lr * 21.663219650436019) + t2 * 158.7883661050584)
               - t3 * 363.5275093004978)
              + t4 * 278.40449096291019)
             + (((((t5 * 188.26692883332171 - t6 * 476.01038424900253) + t7 * 412.121107396029)
                  + K_tmp * 422.72848258265162)
                 - b_K_tmp * 277.14189276484768)
                + c_K_tmp * 207.43746165559941))
          + (((d_K_tmp * -18.087552840893942 - e_K_tmp * 257.74708426633748)
              - f_K_tmp * 213.84390515943051)
             - 2.9352127472027161);
    K[25] = (((((Ll * -115.22431519775991 + Lr * 48.509973787334651) + t2 * 134.91307744957561)
               - t3 * 26.95926535057194)
              - t4 * 99.911831590645136)
             + ((((t5 * -188.4834305308616 + t6 * 325.09785397567919) - t7 * 203.23427914799791)
                 + K_tmp * 53.0098658693353)
                - b_K_tmp * 136.59622052789061))
          + ((((c_K_tmp * -365.73028663947917 + d_K_tmp * 319.229675826918)
               + e_K_tmp * 122.63672565744039)
              + f_K_tmp * 79.900200299195163)
             - 3.099538662189568);
    K[26] = (((((Ll * -31.041791164827259 - Lr * 5.0655366615559041) + t2 * 68.002512287974838)
               - t3 * 66.851331165223968)
              + t4 * 31.586761993210811)
             + ((((t5 * -163.350960274353 + t6 * 683.54431208418237) - t7 * 706.50870664428442)
                 - K_tmp * 410.60760927140058)
                + b_K_tmp * 354.01951313115251))
          + ((((c_K_tmp * 254.75309769186279 - d_K_tmp * 271.50740235131423)
               + e_K_tmp * 199.1223224617313)
              - f_K_tmp * 71.078773685397564)
             + 5.4957791129627873);
    K[27] = ((((((Ll * 166.76454301059709 - Lr * 51.861172099315652) - t2 * 778.42593625807228)
                + t3 * 1531.416724875332)
               - t4 * 1130.798116870355)
              + t5 * 165.1832430027608)
             + ((((t6 * -232.34699361654361 + t7 * 113.7119671644116) - K_tmp * 564.2890557694933)
                 + b_K_tmp * 403.4347555307985)
                - c_K_tmp * 252.13874457596569))
          + (((d_K_tmp * 6.75914758668559 + e_K_tmp * 246.3508376906652)
              + f_K_tmp * 345.33382599365069)
             + 6.7788727396865083);
    K[28] = (((((Ll * -0.18079068073218721 - Lr * 2.3432692727488571) + t2 * 6.4696205342818924)
               - t3 * 16.99415008225802)
              + t4 * 11.72208009408835)
             + ((((t5 * 20.948382651577649 - t6 * 54.573267500271321) + t7 * 49.488296868469227)
                 + K_tmp * 90.200600529027241)
                - b_K_tmp * 67.705083873625867))
          + ((((c_K_tmp * 25.136734249939789 - d_K_tmp * 2.6155683913767822)
               - e_K_tmp * 25.4232951787451)
              - f_K_tmp * 49.80888790715774)
             - 0.23670085285724321);
    K[29] = (((((Ll * -8.94710979774916 + Lr * 3.62189146407981) - t2 * 9.36246819881036)
               + t3 * 18.154635637143329)
              - t4 * 15.464718172453431)
             + ((((t5 * -11.957438367637261 + t6 * 20.382498539823629) - t7 * 11.03924124156781)
                 - K_tmp * 8.3179343764808387)
                - b_K_tmp * 2.5567447780637269))
          + ((((c_K_tmp * -19.652329524928579 + d_K_tmp * 11.80798785612363)
               + e_K_tmp * 12.28372576449847)
              + f_K_tmp * 8.3192505503355569)
             - 0.63087679138113362);
    K[30] = (((((Ll * 2.858622034847341 + Lr * 4.6147258805523412) - t2 * 6.2665206402295563)
               + t3 * 13.09173853094647)
              - t4 * 7.3372458374410252)
             + ((((t5 * -44.6436780067125 + t6 * 152.574028052885) - t7 * 153.04294935261291)
                 - K_tmp * 135.659877850471)
                + b_K_tmp * 124.8019569463824))
          + ((((c_K_tmp * 16.775491469412049 - d_K_tmp * 31.457149506331589)
               + e_K_tmp * 40.13600546271546)
              + f_K_tmp * 17.864998904194341)
             - 0.063289225527233511);
    K[31] = (((((Ll * 6.1271438066735646 - Lr * 4.0761416296436748) - t2 * 21.323907847135551)
               + t3 * 31.725621500309039)
              - t4 * 17.228662231674289)
             + ((((t5 * 14.2689307155937 - t6 * 28.330181285068) + t7 * 16.257584007459581)
                 - K_tmp * 17.92659636802367)
                + b_K_tmp * 24.4901028015961))
          + ((((c_K_tmp * 10.06054963485882 - d_K_tmp * 13.12168111524675)
               - e_K_tmp * 10.870421860065459)
              + f_K_tmp * 13.88755919100582)
             + 1.83783309376778);
    K[32] = ((((((Ll * 138.54481680221721 + Lr * 9.9253351666453487) - t2 * 322.75743211139769)
                + t3 * 326.24780224509988)
               - t4 * 91.948856314512255)
              + t5 * 430.52298315072181)
             + ((((t6 * -1211.093787998286 + t7 * 1021.662505036252) + K_tmp * 216.49677115357659)
                 + b_K_tmp * 91.469612385968873)
                + c_K_tmp * 726.89075356600017))
          + (((d_K_tmp * -516.6038569598212 - e_K_tmp * 385.88493964700092)
              - f_K_tmp * 329.45392685289471)
             - 43.974938188636251);
    K[33] = (((((Ll * 24.840551070207209 + Lr * 123.4084174781029) + t2 * 358.52320472934269)
               - t3 * 977.74103500354875)
              + t4 * 757.980313190083)
             + ((((t5 * -283.74737571255861 + t6 * 199.45384370725881) - t7 * 47.902668128055467)
                 + K_tmp * 919.2242743966176)
                - b_K_tmp * 385.30841593782469))
          + ((((c_K_tmp * -93.643075055204989 + d_K_tmp * 579.18258856320415)
               - e_K_tmp * 770.42609711566774)
              - f_K_tmp * 293.83211745674072)
             - 43.966322549320289);
    K[34] = ((((((Ll * 91.207280228821972 - Lr * 442.48924614593278) - t2 * 374.65945075022569)
                + t3 * 838.19711338330012)
               - t4 * 749.65696876801746)
              + t5 * 958.153179726106)
             + ((((t6 * -1070.449986073746 + t7 * 410.16942237545862) - K_tmp * 623.11263043658141)
                 + b_K_tmp * 439.50343407536951)
                - c_K_tmp * 1191.5150982448361))
          + (((d_K_tmp * 991.83483422592747 + e_K_tmp * 318.22500932278712)
              + f_K_tmp * 615.92718626006069)
             - 51.133163817417874);
    K[35] = ((((((Ll * -495.27755271628968 + Lr * 123.1981622305554) + t2 * 1606.57455678979)
                - t3 * 3139.3800081567219)
               + t4 * 2818.7232453483462)
              - t5 * 147.04519872464451)
             + ((((t6 * 487.32171206076362 - t7 * 246.7041298461703) - K_tmp * 1329.7422628597039)
                 - b_K_tmp * 98.4859753755533)
                + c_K_tmp * 1673.1269142547931))
          + (((d_K_tmp * -3041.8880950636449 + e_K_tmp * 2174.8227874391209)
              - f_K_tmp * 142.52715976870931)
             - 49.937347663985364);
    K[36] = (((((Ll * 25.50072511648553 - Lr * 2.9751540526313489) - t2 * 70.858766012158014)
               + t3 * 92.965568754329439)
              - t4 * 45.564233261422828)
             + ((((t5 * 61.455327698036648 - t6 * 148.22284161247629) + t7 * 116.47307835400549)
                 + K_tmp * 32.86952080699475)
                + b_K_tmp * 10.077812291865239))
          + ((((c_K_tmp * 129.2590538779566 - d_K_tmp * 98.258228910174267)
               - e_K_tmp * 57.8997277977946)
              - f_K_tmp * 55.583997249577592)
             - 5.0490233241243381);
    K[37] = (((((Ll * -1.307776114470145 + Lr * 23.6743531404337) + t2 * 56.622959777982828)
               - t3 * 142.45505833078309)
              + t4 * 113.9134215869385)
             + ((((t5 * -65.240240959539179 + t6 * 85.753001084681713) - t7 * 45.1409547275459)
                 + K_tmp * 123.0408597323801)
                - b_K_tmp * 80.962744258010375))
          + ((((c_K_tmp * 37.719437635966962 + d_K_tmp * 11.02197945335028)
               - e_K_tmp * 71.335378510101179)
              - f_K_tmp * 55.326587372136693)
             - 5.0407199143961208);
    K[38] = (((((Ll * -5.7193075357334013 - Lr * 32.56474930130765) + t2 * 16.72861612812018)
               - t3 * 18.108047122543461)
              + t4 * 5.589289848391144)
             + ((((t5 * 25.065550786458481 + t6 * 63.460716411429772) - t7 * 98.269508679281941)
                 - K_tmp * 141.15058240140769)
                + b_K_tmp * 70.9314021231647))
          + ((((c_K_tmp * -153.56053985785061 + d_K_tmp * 90.215499630540876)
               + e_K_tmp * 113.9370029039823)
              + f_K_tmp * 95.74661236667599)
             - 1.059057736683205);
    K[39] = (((((Ll * -36.240250276701971 - Lr * 2.3479636045503511) + t2 * 55.66522399137196)
               - t3 * 18.298114652492131)
              - t4 * 14.5668623360618)
             + ((((t5 * 16.9979338880978 - t6 * 9.2485441614476915) + t7 * 3.3751682461684789)
                 - K_tmp * 163.60154058698851)
                + b_K_tmp * 60.589146226320842))
          + ((((c_K_tmp * -61.082091812454308 - d_K_tmp * 38.536633460519212)
               + e_K_tmp * 172.974109418491)
              + f_K_tmp * 66.443317384104859)
             - 1.041441249495821);
}

} // namespace app::controller