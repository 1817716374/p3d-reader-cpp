// Source attribute schemas selected by the native M634 procedure type.
// Opaque M-names are retained; these schemas do not define shader equations.
static const ProcedureFields procedure_fields[] = {
    {"", "", ""}, // 0: no factory type
    {"Alpha1 Alpha2 AmplitudeRatio Bias FrequencyRatio Gain M155 M156", "M711 M712",
     "Color1 Color2"},                                                   // 1
    {"Alpha1 Alpha2 Bias Gain M114 M155 M156", "M113", "Color1 Color2"}, // 2
    {"Bias Gain M114 M117 M118 M119 M157 M158", "M120", "M115 M116"},    // 3
    {"Bias Gain M114 M118 M122 M123 M158 M161", "M124", "M116 M121"},    // 4
    {"M155", "", "M125"},                                                // 5
    {"AmplitudeRatio Bias FrequencyRatio Gain M114 M118 M127 M128 M129 M130 M131 M158 M167",
     "M132 M711", "M116 M126"},                                                  // 6
    {"Bias Gain M118 M134 M135 M136 M137 M138 M139 M158 M162", "", "M116 M133"}, // 7
    {"Bias Gain M142 M143 M144 M145 M163 M164", "", "M140 M141"},                // 8
    {"Bias Gain M148 M151 M152 M153 M165 M166", "M154", "M146 M147"},            // 9
    {"", "M218", ""},                                                            // 10
    {"M139 M425 M426 M427", "M428 M429 M430", "M133 M424"},                      // 11
    {"M426 M427 M433 M434 M460", "M430 M435 M436 M437", "M431 M432"},            // 12
    {"", "", ""},                                                                // 13
    {"M438", "", "Color1 Color2"},                                               // 14
    {"M438", "", "Color1 Color2"},                                               // 15
    {"M441 M442 M443", "M444", "M439 M440"},                                     // 16
    {"", "", ""},                                                                // 17
    {"M442 M445 M446 M447 M451", "", "M125"},                                    // 18
    {"M441 M442 M526 M527 M528 M529 M530", "", "M525"},                          // 19
    {"M442 M454", "", "M452 M453"},                                              // 20
    {"", "", ""},                                                                // 21
    {"M456", "", "M455 M512"},                                                   // 22
    {"M433 M460", "M458 M459", "M432 M457"},                                     // 23
    {"M442", "", ""},                                                            // 24
    {"", "", "M511 M512"},                                                       // 25
    {"M136 M514 M515 M516 M521 M522 M523", "", "M513"},                          // 26
    {"", "", "M512 M534"},                                                       // 27
    {"", "", "M133 M424"},                                                       // 28
    {"", "", "M133 M424"},                                                       // 29
    {"Alpha1 Alpha2 Bias Gain M155 M156 M314 M315 M316 M317 M318", "M421 M422 M423",
     "Color1 Color2"},                                                                   // 30
    {"", "", ""},                                                                        // 31
    {"Bias Gain M21 M22 M26 M27 M28 M29 NoiseSeed", "Absolute M30 M31 M713", "M24 M25"}, // 32
    {"Bias Gain M1 M21 M22 M26 M27 M28 M29 M5 M9 NoiseSeed", "Absolute M30 M31 M713",
     "M24 M25"},                                                                       // 33
    {"Bias Gain M14 M21 M22 M26 M27 M28 M29 M5 NoiseSeed", "M30 M31 M713", "M24 M25"}, // 34
    {"Bias Gain M1 M17 M21 M22 M26 M27 M28 M29 NoiseSeed", "M30 M31 M713", "M24 M25"}, // 35
    {"Bias Gain M1 M18 M21 M22 M26 M27 M28 M29 NoiseSeed", "M30 M31 M713", "M24 M25"}, // 36
    {"Bias Gain M1 M19 M20 M21 M22 M26 M27 M28 M29 NoiseSeed", "Absolute FlowType M30 M31",
     "M24 M25"}, // 37
    {"Bias Gain M1 M21 M22 M26 M27 M28 M29 M5 M9 NoiseSeed", "Absolute M30 M31 M713",
     "M24 M25"},                                                                           // 38
    {"Bias Gain M1 M21 M22 M26 M27 M28 M29 M5 NoiseSeed", "Fold M30 M31 M713", "M24 M25"}, // 39
    {"Bias Gain M1 M11 M21 M22 M26 M27 M28 M29 M5 M9 NoiseSeed", "Heterogeneous M30 M31 M713",
     "M24 M25"},                                                                             // 40
    {"Bias Gain M1 M21 M22 M26 M27 M28 M29 M5 NoiseSeed", "M30 M31 M713", "M24 M25"},        // 41
    {"Bias Gain M1 M21 M22 M26 M27 M28 M29 M5 M9 NoiseSeed", "M30 M31 M713", "M24 M25"},     // 42
    {"Bias Gain M1 M10 M21 M22 M26 M27 M28 M29 M5 M9 NoiseSeed", "M30 M31 M713", "M24 M25"}, // 43
    {"Bias Gain M1 M11 M21 M22 M26 M27 M28 M29 M5 M9 NoiseSeed", "M30 M31 M713", "M24 M25"}, // 44
    {"Bias Gain M14 M21 M22 M26 M27 M28 M29 M5 M6 M7 M8 NoiseSeed", "M30 M31 M713",
     "M24 M25"}, // 45
    {"Bias Gain M1 M13 M21 M22 M26 M27 M28 M29 M5 M9 NoiseSeed", "Heterogeneous M30 M31 M713",
     "M24 M25"}, // 46
    {"Bias Gain M1 M11 M12 M21 M22 M26 M27 M28 M29 M5 M9 NoiseSeed", "M30 M31 M713",
     "M24 M25"},                                                                         // 47
    {"Bias Gain M1 M21 M22 M26 M27 M28 M29 M5 M9 NoiseSeed", "M30 M31 M713", "M24 M25"}, // 48
    {"Bias Gain M13 M2 M21 M22 M26 M27 M28 M29 NoiseSeed", "AxisType M30 M31 M713",
     "M24 M25"}, // 49
    {"Bias Gain M1 M14 M21 M22 M26 M27 M28 M29 M3 M5 M9 NoiseSeed", "AxisType M30 M31 M713",
     "M24 M25"},                                                                            // 50
    {"Bias Gain M1 M21 M22 M26 M27 M28 M29 M4 M5 M9 NoiseSeed", "M30 M31 M713", "M24 M25"}, // 51
    {"Bias Gain M1 M14 M21 M22 M26 M27 M28 M29 M5 NoiseSeed", "Fractalize M30 M31 M713",
     "M24 M25"},                                                                          // 52
    {"Bias Gain M1 M14 M21 M22 M26 M27 M28 M29 M5 NoiseSeed", "M30 M31 M713", "M24 M25"}, // 53
    {"M81 M84 M85 M86 M87", "", "M82 M83"},                                               // 54
    {"Bias Gain M1 M14 M15 M16 M26 M27 M28 M29 M32 M33 M34 M41 M42 M43 M44 M79 M80 M81 NoiseSeed",
     "Full3D M30 M31 M713", "M24 M25"}, // 55
    {"M1 M14 M15 M16 M26 M27 M28 M29 M32 M33 M34 M35 M79 M80 M81 NoiseSeed",
     "Full3D M23 M30 M31 M713", "M24 M25"}, // 56
    {"Bias Gain M1 M14 M15 M16 M26 M27 M28 M29 M32 M33 M34 M45 M48 M51 M52 M53 M79 M80 M81 "
     "NoiseSeed",
     "Full3D M23 M30 M31 M54 M55 M56 M57 M58 M59 M713", "M24 M25"}, // 57
    {"Bias Gain M1 M14 M15 M16 M26 M27 M28 M29 M32 M33 M34 M45 M48 M49 M50 M53 M79 M80 M81 "
     "NoiseSeed",
     "Full3D M23 M30 M31 M54 M55 M56 M57 M58 M59 M713", "M24 M25"}, // 58
    {"Bias Gain M1 M14 M15 M16 M26 M27 M28 M29 M32 M33 M34 M46 M47 M52 M79 M80 M81 NoiseSeed",
     "Full3D M30 M31 M713", "M24 M25"}, // 59
    {"Bias Gain M1 M14 M15 M16 M26 M27 M28 M29 M32 M33 M34 M60 M61 M62 M79 M80 M81 NoiseSeed",
     "Full3D M30 M31 M63 M713", "M24 M25"}, // 60
    {"M1 M14 M15 M16 M26 M27 M28 M29 M66 M67 M68 M69 M70 M72 M73 M74 M76 M77 M79 M80 NoiseSeed",
     "Full3D M23 M30 M31 M64 M65 M71 M713 M75", "M24 M25"}, // 61
    {"M1 M14 M15 M16 M26 M27 M28 M29 M33 M34 M35 M78 M79 M80 M81 NoiseSeed",
     "Full3D M23 M30 M31 M713", "M24 M25"}, // 62
    {"M1 M14 M15 M16 M26 M27 M28 M29 M33 M34 M35 M79 M80 M81 NoiseSeed", "Full3D M23 M30 M31 M713",
     "M24 M25"}, // 63
    {"M1 M14 M15 M16 M26 M27 M28 M29 M33 M36 M79 M80 M81 NoiseSeed", "Full3D M23 M30 M31 M713",
     "M24 M25"}, // 64
    {"M1 M14 M15 M16 M26 M27 M28 M29 M33 M34 M35 M79 M80 M81 NoiseSeed",
     "Full3D M23 M30 M31 M37 M713", "M24 M25"}, // 65
    {"M1 M14 M15 M16 M2 M26 M27 M28 M29 M33 M34 M35 M78 M79 M80 M81 NoiseSeed",
     "Full3D M23 M30 M31 M38 M713", "M24 M25"}, // 66
    {"M1 M14 M145 M15 M16 M26 M27 M28 M29 M33 M34 M35 M79 M80 M81 NoiseSeed",
     "Full3D M23 M30 M31 M39 M40 M713", "M24 M25"},                                 // 67
    {"Bias Gain M2 M20 M21 M22 M26 M27 M28 M29 M81 M90", "M30 M31 M88", "M24 M25"}, // 68
    {"Bias Gain M2 M20 M21 M22 M26 M27 M28 M29 M89 M90", "M30 M31", "M24 M25"},     // 69
    {"Bias Gain M100 M111 M112 M21 M22 M26 M27 M28 M29 M92 M93 M94 M95 M96 M98 M99",
     "M30 M31 M91 M97", "M24 M25"}, // 70
};
