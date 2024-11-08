//  Copyright (c) 2024 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

/* eslint-disable default-case */
/* eslint-disable max-len */

import {
  LerpLookupTable,
  LerpVectorLookupTable,
  MathUtils,
  ReadonlyFloat64Array,
  Vec2Math,
  Vec3Math,
  VecNMath,
} from '@microsoft/msfs-sdk';

import {
  LineupAngle,
  TakeoffPerformanceCalculator,
  TakeoffPerfomanceError,
  TakeoffPerformanceInputs,
  TakeoffPerformanceParameters,
  TakeoffPerformanceResult,
  TakeoffAntiIceSetting,
  RunwayCondition,
  LimitingFactor,
  LimitWeight,
  TakeoffPerformanceSpeeds,
} from '@flybywiresim/fbw-sdk';

/**
 * Takeoff performance calculator for an A320-251N with forward CG
 */
export class A380482xTakeoffPerformanceCalculator implements TakeoffPerformanceCalculator {
  private static readonly vec2Cache = Vec2Math.create();
  private static readonly vec3Cache = Vec3Math.create();
  private static readonly vec4Cache = VecNMath.create(4);

  private static resultCache: Partial<TakeoffPerformanceResult> = {};

  private static optResultCache: Partial<TakeoffPerformanceResult>[] = [{}, {}, {}];

  /** Max flex temp as a delta from ISA in °C. */
  private static readonly tMaxFlexDisa = 59;

  public readonly structuralMtow = 512_000;

  public readonly maxPressureAlt = 9_200;

  public readonly oew = 275_443;

  public readonly maxHeadwind = 45;

  public readonly maxTailwind = 15;

  /** Lineup distance for each lineup angle, in metres. */
  private static readonly lineUpDistances: Record<LineupAngle, number> = {
    0: 0,
    90: 20.5,
    180: 41,
  };

  /** Tref lookup table, (Tref [°C], elevation [feet]), lookup key = (elevation) */
  private static readonly tRefTable = new LerpLookupTable([
    [48, -2000],
    [44, 0],
    [43, 500],
    [42, 1000],
    [40, 2000],
    [36.4, 3000],
    [35, 3479],
    [16.4, 8348],
    [13.5, 9200],
  ]);

  /** Tmax lookup table, Tmax [°C], pressure alt [feet] => lookup key = (pressure alt) */
  private static readonly tMaxTable = new LerpLookupTable([
    [55, -2000],
    [55, 0],
    [38, 9200],
  ]);

  /** CONF 1+F runway limited weights at sea level/ISA/0 slope/no bleed/fwd cg/no wind/dry, MTOW [kg], runway length [metres] => lookup key = (runway length) */
  private static readonly runwayPerfLimitConf1 = new LerpLookupTable([
    [349_975, 1000],
    [384_324, 1219],
    [445_246, 1604],
    [490_613, 1959],
    [512_000, 2134],
    [522_370, 2239],
    [542_461, 2459],
    [550_886, 2559],
    [558_663, 2709],
    [562_876, 2918],
    [576_810, 3000],
    [587_828, 3180],
    [659_119, 3800],
    [699_949, 5000],
  ]);

  /** CONF 2 runway limited weights at sea level/ISA/0 slope/no bleed/fwd cg/no wind/dry, MTOW [kg], runway length [metres] => lookup key = (runway length) */
  private static readonly runwayPerfLimitConf2 = new LerpLookupTable([
    [353_215, 1000],
    [392_101, 1219],
    [460_152, 1604],
    [508_111, 1959],
    [526_906, 2134],
    [537_276, 2239],
    [554_127, 2459],
    [565_792, 2709],
    [570_329, 2879],
    [571_107, 2987],
    [619_585, 3600],
    [629_954, 3800],
    [634_491, 3900],
    [686_987, 5000],
  ]);

  /** CONF 3 runway limited weights at sea level/ISA/0 slope/no bleed/fwd cg/no wind/dry, MTOW [kg], runway length [metres] => lookup key = (runway length) */
  private static readonly runwayPerfLimitConf3 = new LerpLookupTable([
    [344_142, 1000],
    [390_157, 1219],
    [470_522, 1604],
    [517_833, 1959],
    [539_220, 2134],
    [541_165, 2239],
    [551_534, 2459],
    [569_033, 2709],
    [575_514, 2839],
    [594_309, 3180],
    [632_547, 3800],
    [680_506, 5000],
  ]);

  /** Slope factor for each takeoff config. */
  private static readonly runwaySlopeFactor: Record<number, number> = {
    1: 0.00084,
    2: 0.00096,
    3: 0.0011,
  };

  /** Pressure altitude factors for each takeoff config. */
  private static readonly runwayPressureAltFactor: Record<number, [number, number]> = {
    1: [3.43e-8, 0.001192],
    2: [1.15e-8, 0.001216],
    3: [-4.6e-9, 0.001245],
  };

  /** Temperature factors for each takeoff config. */
  private static readonly runwayTemperatureFactor: Record<number, [number, number, number, number, number, number]> = {
    1: [0.00001, 0.095175, 0.000207, 0.040242, 0.00024, 0.066189],
    2: [-0.00001, 0.131948, 0.000155, 0.162938, 0.000225, 0.150363],
    3: [-0.0000438, 0.198845, 0.000188, 0.14547, 0.0002, 0.232529],
  };

  /** Headwind factors for each takeoff config. */
  private static readonly runwayHeadWindFactor: Record<number, [number, number, number, number]> = {
    1: [0.000029, -0.233075, 0.00242, 0.003772],
    2: [0.000051, -0.277863, 0.0018, 0.003366],
    3: [0.000115, -0.3951, 0.002357, 0.002125],
  };

  /** Tailwind factors for each takeoff config. */
  private static readonly runwayTailWindFactor: Record<number, [number, number, number, number]> = {
    1: [0.000065, -0.684701, 0.00498, 0.0808],
    2: [0.000198, -1.017, 0.00711, 0.009],
    3: [0.000271, -1.11506, 0.0078, 0.00875],
  };

  /** Segment segment weight factors for each takeoff config. */
  private static readonly secondSegmentBaseFactor: Record<number, [number, number]> = {
    1: [0.00391, 75.366],
    2: [0.005465, 72.227],
    3: [0.00495, 72.256],
  };

  /** Slope factor for each takeoff config. */
  private static readonly secondSegmentSlopeFactor: Record<number, number> = {
    1: 0.000419,
    2: 0.000641,
    3: 0.000459,
  };

  /** Pressure altitude factors for each takeoff config. */
  private static readonly secondSegmentPressureAltFactor: Record<number, [number, number]> = {
    1: [-6.5e-8, 0.001769],
    2: [1.05e-7, 0.00055],
    3: [7.48e-8, 0.000506],
  };

  /** Temperature factors for each takeoff config. */
  private static readonly secondSegmentTemperatureFactor: Record<
    number,
    [number, number, number, number, number, number]
  > = {
    1: [0.000025, 0.001, 0.000155, 0.211445, 0.000071, 0.556741],
    2: [0.0000121, 0.042153, 0.0001256, 0.325925, 0.000082, 0.546259],
    3: [-0.0000294, 0.13903, 0.0000693, 0.480536, 0.000133, 0.480536],
  };

  /** Headwind factors for each takeoff config. */
  private static readonly secondSegmentHeadWindFactor: Record<number, [number, number, number, number]> = {
    1: [0.000019, -0.13052, 0.000813636, 0.000145238],
    2: [0.0000454, -0.20585, 0.000416667, 0.001778293],
    3: [0.000085, -0.30209, 0.001189394, 0.0038996],
  };

  /** Tailwind factors for each takeoff config. */
  private static readonly secondSegmentTailWindFactor: Record<number, [number, number, number, number]> = {
    1: [0.000104, -0.705693, 0.009, 0.00648],
    2: [0.000154, -0.8052, 0.009, 0.002444],
    3: [0.000054, -0.462, 0.00875, 0.006606505],
  };

  /** Segment segment weight factors for each takeoff config. */
  private static readonly brakeEnergyBaseFactor: Record<number, [number, number]> = {
    1: [0.00503, 72.524],
    2: [0.00672, 68.28],
    3: [0.00128, 83.951],
  };

  /** Slope factor for each takeoff config. */
  private static readonly brakeEnergySlopeFactor: Record<number, number> = {
    1: 0.000045,
    2: 0.000068,
    3: 0.000045,
  };

  /** Pressure altitude factors for each takeoff config. */
  private static readonly brakeEnergyPressureAltFactor: Record<number, [number, number]> = {
    1: [5.5e-8, 0.000968],
    2: [1.17e-7, 0.000595],
    3: [4.65e-8, 0.000658],
  };

  /** Temperature factors for each takeoff config. */
  private static readonly brakeEnergyTemperatureFactor: Record<number, [number, number]> = {
    1: [0.06, 0.54],
    2: [0.058, 0.545],
    3: [0.04642, 0.6],
  };

  /** Headwind factors for each takeoff config. */
  private static readonly brakeEnergyHeadWindFactor: Record<number, [number, number, number, number]> = {
    1: [0.0000311, -0.1769, 0.001125, 0],
    2: [0.0000316, -0.1799, 0.001182, 0],
    3: [0.0000147, -0.0928, 0.001111, 0],
  };

  /** Tailwind factors for each takeoff config. */
  private static readonly brakeEnergyTailWindFactor: Record<number, [number, number, number, number]> = {
    1: [0.000117, -0.8024, 0.0117879, 0.006667],
    2: [0.000157, -0.849, 0.0066818, 0.006667],
    3: [0.00013, -0.6946, 0.0068333, 0.006667],
  };

  /** Segment segment weight factors for each takeoff config. */
  private static readonly vmcgBaseFactor: Record<number, [number, number]> = {
    1: [0.0644, -19.526],
    2: [0.082005, -39.27],
    3: [0.0704, -25.6868],
  };

  /** Slope factor for each takeoff config. */
  private static readonly vmcgSlopeFactor: Record<number, number> = {
    1: 0.00084,
    2: 0.001054,
    3: 0.001068,
  };

  /** Pressure altitude factors for each takeoff config. */
  private static readonly vmcgPressureAltFactor: Record<number, [number, number]> = {
    1: [-8.35e-7, 0.00589],
    2: [-7.58e-7, 0.00703],
    3: [1.95e-7, 0.00266],
  };

  /** Temperature factors for each takeoff config. */
  private static readonly vmcgTemperatureFactor: Record<number, [number, number, number, number, number, number]> = {
    1: [-0.00133, 2.104, 0.000699, -0.128144, -0.000718, 1.8103],
    2: [-0.00097, 1.613, 0.000242, 0.462005, -0.000547, 1.603],
    3: [-0.000923, 1.6087, 0.00061, 0.002239, -0.000335, 1.2716],
  };

  /** Headwind factors for each takeoff config. */
  private static readonly vmcgHeadWindFactor: Record<
    number,
    [number, number, number, number, number, number, number, number]
  > = {
    1: [0.001198, -1.80539, 0.000097, -0.15121, -0.000255, 0.337391, 0.000066, -0.079718],
    2: [0.000697, -1.17473, 0.000031, -0.057504, -0.000184, 0.246185, 0.000012, 0.0216],
    3: [0.0023, -3.468, -0.000037, 0.033946, -0.000156, 0.213953, -0.000757, 1.094],
  };

  /** Tailwind factors for each takeoff config. */
  private static readonly vmcgTailWindFactor: Record<number, [number, number, number, number, number, number]> = {
    1: [0.00218, -5.489, -0.000106, 0.145473, 0.031431, -0.0356],
    2: [0.001892, -5.646, -0.000059, 0.079539, 0.009948, -0.010763],
    3: [0.000613, -3.165, -0.000022, 0.020622, 0.049286, -0.0396],
  };

  /** Takeoff CG envelope. key = TOW [kg] => [lower limit, upper limit] %MAC. */
  private static readonly takeoffCgLimits = new LerpVectorLookupTable([
    [Vec2Math.create(15, 32.5), 262_481],
    [Vec2Math.create(15, 37), 324_051],
    [Vec2Math.create(15, MathUtils.lerp(343_494, 324_051, 435_524, 37, 40)), 343_494],
    [Vec2Math.create(17, MathUtils.lerp(408_304, 324_051, 435_524, 37, 40)), 408_304],
    [Vec2Math.create(17, 40), 435_524],
    [Vec2Math.create(17, 40), 466_633],
    [Vec2Math.create(MathUtils.lerp(474_410, 466_633, 499_038, 17, 24), 40), 474_410],
    [Vec2Math.create(24, 37.2), 499_038],
  ]);

  private static readonly cgFactors: Record<number, [number, number]> = {
    1: [-0.041448, 3.357],
    2: [-0.03277, 2.686],
    3: [-0.0249, 2.086],
  };

  /** Minimum V1 limited by VMCG/VMCA tables... lookup key is pressure altitude [feet], value is kcas. The speeds are the same for all configs. */
  private static readonly minimumV1Vmc = new LerpLookupTable([
    [122, -2000],
    [121, 0],
    [121, 2000],
    [120, 3000],
    [120, 4000],
    [118, 6000],
    [116, 8000],
    [115, 9200],
    [107, 14000],
    [106, 15000],
  ]);

  /** Minimum Vr limited by VMCG/VMCA tables... lookup key is pressure altitude [feet], value is kcas. The speeds are the same for all configs. */
  private static readonly minimumVrVmc = new LerpLookupTable([
    [123, -2000],
    [122, 0],
    [122, 3000],
    [121, 4000],
    [120, 6000],
    [117, 8000],
    [116, 9200],
    [107, 14000],
    [106, 15000],
  ]);

  /** Minimum V2 limited by VMCG/VMCA tables... outer key is takeoff config, lookup key is pressure altitude [feet], value is kcas. */
  private static readonly minimumV2Vmc: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [127, -2000],
      [126, 0],
      [126, 1000],
      [125, 2000],
      [125, 3000],
      [124, 4000],
      [123, 6000],
      [120, 8000],
      [118, 9200],
      [109, 14000],
      [107, 15000],
    ]),
    2: new LerpLookupTable([
      [127, -2000],
      [126, 0],
      [126, 1000],
      [125, 2000],
      [125, 3000],
      [124, 4000],
      [123, 6000],
      [120, 8000],
      [118, 9200],
      [109, 14000],
      [107, 15000],
    ]),
    3: new LerpLookupTable([
      [126, -2000],
      [125, 0],
      [125, 1000],
      [124, 2000],
      [124, 3000],
      [123, 4000],
      [122, 6000],
      [119, 8000],
      [117, 9200],
      [108, 14000],
      [106, 15000],
    ]),
  };

  /** Minimum V2 limited by VMU/VMCA tables... outer key is takeoff config, lookup keys are (pressure altitude [feet], takeoff weight [kg]), value is kcas. */
  private static readonly minimumV2Vmu: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [127, -2000, 291_646],
      [127, -2000, 324_051],
      [127, -2000, 356_456],
      [132, -2000, 388_861],
      [137, -2000, 421_266],
      [142, -2000, 453_671],
      [146, -2000, 486_076],
      [151, -2000, 518_481],
      [126, -1000, 291_646],
      [126, -1000, 324_051],
      [127, -1000, 356_456],
      [132, -1000, 388_861],
      [137, -1000, 421_266],
      [142, -1000, 453_671],
      [147, -1000, 486_076],
      [151, -1000, 518_481],
      [126, 0, 291_646],
      [126, 0, 324_051],
      [127, 0, 356_456],
      [132, 0, 388_861],
      [137, 0, 421_266],
      [142, 0, 453_671],
      [147, 0, 486_076],
      [151, 0, 518_481],
      [126, 1000, 291_646],
      [126, 1000, 324_051],
      [127, 1000, 356_456],
      [132, 1000, 388_861],
      [137, 1000, 421_266],
      [142, 1000, 453_671],
      [147, 1000, 486_076],
      [151, 1000, 518_481],
      [125, 2000, 291_646],
      [125, 2000, 324_051],
      [127, 2000, 356_456],
      [132, 2000, 388_861],
      [137, 2000, 421_266],
      [142, 2000, 453_671],
      [147, 2000, 486_076],
      [151, 2000, 518_481],
      [125, 3000, 291_646],
      [125, 3000, 324_051],
      [127, 3000, 356_456],
      [132, 3000, 388_861],
      [137, 3000, 421_266],
      [142, 3000, 453_671],
      [147, 3000, 486_076],
      [151, 3000, 518_481],
      [124, 4000, 291_646],
      [124, 4000, 324_051],
      [127, 4000, 356_456],
      [132, 4000, 388_861],
      [137, 4000, 421_266],
      [142, 4000, 453_671],
      [147, 4000, 486_076],
      [152, 4000, 518_481],
      [124, 5000, 291_646],
      [124, 5000, 324_051],
      [127, 5000, 356_456],
      [132, 5000, 388_861],
      [137, 5000, 421_266],
      [142, 5000, 453_671],
      [147, 5000, 486_076],
      [152, 5000, 518_481],
      [123, 6000, 291_646],
      [123, 6000, 324_051],
      [127, 6000, 356_456],
      [132, 6000, 388_861],
      [137, 6000, 421_266],
      [142, 6000, 453_671],
      [147, 6000, 486_076],
      [152, 6000, 518_481],
      [122, 7000, 291_646],
      [122, 7000, 324_051],
      [127, 7000, 356_456],
      [132, 7000, 388_861],
      [137, 7000, 421_266],
      [142, 7000, 453_671],
      [147, 7000, 486_076],
      [152, 7000, 518_481],
      [120, 8000, 291_646],
      [121, 8000, 324_051],
      [127, 8000, 356_456],
      [132, 8000, 388_861],
      [137, 8000, 421_266],
      [143, 8000, 453_671],
      [148, 8000, 486_076],
      [152, 8000, 518_481],
      [119, 9000, 291_646],
      [121, 9000, 324_051],
      [127, 9000, 356_456],
      [132, 9000, 388_861],
      [137, 9000, 421_266],
      [143, 9000, 453_671],
      [148, 9000, 486_076],
      [153, 9000, 518_481],
      [117, 10000, 291_646],
      [121, 10000, 324_051],
      [127, 10000, 356_456],
      [132, 10000, 388_861],
      [137, 10000, 421_266],
      [143, 10000, 453_671],
      [148, 10000, 486_076],
      [153, 10000, 518_481],
      [115, 11000, 291_646],
      [121, 11000, 324_051],
      [127, 11000, 356_456],
      [132, 11000, 388_861],
      [138, 11000, 421_266],
      [143, 11000, 453_671],
      [149, 11000, 486_076],
      [154, 11000, 518_481],
      [115, 12000, 291_646],
      [121, 12000, 324_051],
      [127, 12000, 356_456],
      [132, 12000, 388_861],
      [138, 12000, 421_266],
      [143, 12000, 453_671],
      [149, 12000, 486_076],
      [154, 12000, 518_481],
      [115, 13000, 291_646],
      [121, 13000, 324_051],
      [127, 13000, 356_456],
      [132, 13000, 388_861],
      [138, 13000, 421_266],
      [144, 13000, 453_671],
      [149, 13000, 486_076],
      [154, 13000, 518_481],
      [115, 14100, 291_646],
      [121, 14100, 324_051],
      [127, 14100, 356_456],
      [132, 14100, 388_861],
      [138, 14100, 421_266],
      [144, 14100, 453_671],
      [150, 14100, 486_076],
      [155, 14100, 518_481],
      [115, 15100, 291_646],
      [121, 15100, 324_051],
      [127, 15100, 356_456],
      [133, 15100, 388_861],
      [139, 15100, 421_266],
      [144, 15100, 453_671],
      [150, 15100, 486_076],
      [155, 15100, 518_481],
    ]),
    2: new LerpLookupTable([
      [127, -2000, 291_646],
      [127, -2000, 324_051],
      [127, -2000, 356_456],
      [127, -2000, 388_861],
      [132, -2000, 421_266],
      [136, -2000, 453_671],
      [141, -2000, 486_076],
      [145, -2000, 518_481],
      [126, -1000, 291_646],
      [126, -1000, 324_051],
      [126, -1000, 356_456],
      [127, -1000, 388_861],
      [132, -1000, 421_266],
      [136, -1000, 453_671],
      [141, -1000, 486_076],
      [145, -1000, 518_481],
      [126, 0, 291_646],
      [126, 0, 324_051],
      [126, 0, 356_456],
      [127, 0, 388_861],
      [132, 0, 421_266],
      [137, 0, 453_671],
      [141, 0, 486_076],
      [146, 0, 518_481],
      [126, 1000, 291_646],
      [126, 1000, 324_051],
      [126, 1000, 356_456],
      [127, 1000, 388_861],
      [132, 1000, 421_266],
      [137, 1000, 453_671],
      [141, 1000, 486_076],
      [146, 1000, 518_481],
      [125, 2000, 291_646],
      [125, 2000, 324_051],
      [125, 2000, 356_456],
      [127, 2000, 388_861],
      [132, 2000, 421_266],
      [137, 2000, 453_671],
      [141, 2000, 486_076],
      [146, 2000, 518_481],
      [125, 3000, 291_646],
      [125, 3000, 324_051],
      [125, 3000, 356_456],
      [127, 3000, 388_861],
      [132, 3000, 421_266],
      [137, 3000, 453_671],
      [142, 3000, 486_076],
      [146, 3000, 518_481],
      [124, 4000, 291_646],
      [124, 4000, 324_051],
      [124, 4000, 356_456],
      [127, 4000, 388_861],
      [132, 4000, 421_266],
      [137, 4000, 453_671],
      [142, 4000, 486_076],
      [146, 4000, 518_481],
      [124, 5000, 291_646],
      [124, 5000, 324_051],
      [124, 5000, 356_456],
      [127, 5000, 388_861],
      [132, 5000, 421_266],
      [137, 5000, 453_671],
      [142, 5000, 486_076],
      [146, 5000, 518_481],
      [123, 6000, 291_646],
      [123, 6000, 324_051],
      [123, 6000, 356_456],
      [127, 6000, 388_861],
      [132, 6000, 421_266],
      [137, 6000, 453_671],
      [142, 6000, 486_076],
      [146, 6000, 518_481],
      [122, 7000, 291_646],
      [122, 7000, 324_051],
      [122, 7000, 356_456],
      [127, 7000, 388_861],
      [132, 7000, 421_266],
      [137, 7000, 453_671],
      [142, 7000, 486_076],
      [146, 7000, 518_481],
      [120, 8000, 291_646],
      [120, 8000, 324_051],
      [122, 8000, 356_456],
      [127, 8000, 388_861],
      [132, 8000, 421_266],
      [137, 8000, 453_671],
      [142, 8000, 486_076],
      [146, 8000, 518_481],
      [119, 9000, 291_646],
      [119, 9000, 324_051],
      [122, 9000, 356_456],
      [127, 9000, 388_861],
      [132, 9000, 421_266],
      [137, 9000, 453_671],
      [142, 9000, 486_076],
      [147, 9000, 518_481],
      [117, 10000, 291_646],
      [117, 10000, 324_051],
      [122, 10000, 356_456],
      [127, 10000, 388_861],
      [132, 10000, 421_266],
      [137, 10000, 453_671],
      [142, 10000, 486_076],
      [147, 10000, 518_481],
      [115, 11000, 291_646],
      [117, 11000, 324_051],
      [122, 11000, 356_456],
      [127, 11000, 388_861],
      [132, 11000, 421_266],
      [137, 11000, 453_671],
      [142, 11000, 486_076],
      [147, 11000, 518_481],
      [113, 12000, 291_646],
      [117, 12000, 324_051],
      [122, 12000, 356_456],
      [127, 12000, 388_861],
      [132, 12000, 421_266],
      [138, 12000, 453_671],
      [143, 12000, 486_076],
      [147, 12000, 518_481],
      [111, 13000, 291_646],
      [116, 13000, 324_051],
      [122, 13000, 356_456],
      [127, 13000, 388_861],
      [133, 13000, 421_266],
      [138, 13000, 453_671],
      [143, 13000, 486_076],
      [148, 13000, 518_481],
      [111, 14100, 291_646],
      [116, 14100, 324_051],
      [122, 14100, 356_456],
      [127, 14100, 388_861],
      [133, 14100, 421_266],
      [138, 14100, 453_671],
      [143, 14100, 486_076],
      [148, 14100, 518_481],
      [111, 15100, 291_646],
      [116, 15100, 324_051],
      [122, 15100, 356_456],
      [127, 15100, 388_861],
      [133, 15100, 421_266],
      [138, 15100, 453_671],
      [143, 15100, 486_076],
      [148, 15100, 518_481],
    ]),
    3: new LerpLookupTable([
      [126, -2000, 291_646],
      [126, -2000, 324_051],
      [126, -2000, 356_456],
      [126, -2000, 388_861],
      [128, -2000, 421_266],
      [132, -2000, 453_671],
      [137, -2000, 486_076],
      [141, -2000, 518_481],
      [125, -1000, 291_646],
      [125, -1000, 324_051],
      [125, -1000, 356_456],
      [125, -1000, 388_861],
      [128, -1000, 421_266],
      [132, -1000, 453_671],
      [137, -1000, 486_076],
      [141, -1000, 518_481],
      [125, 0, 291_646],
      [125, 0, 324_051],
      [125, 0, 356_456],
      [125, 0, 388_861],
      [128, 0, 421_266],
      [132, 0, 453_671],
      [137, 0, 486_076],
      [141, 0, 518_481],
      [125, 1000, 291_646],
      [125, 1000, 324_051],
      [125, 1000, 356_456],
      [125, 1000, 388_861],
      [128, 1000, 421_266],
      [132, 1000, 453_671],
      [137, 1000, 486_076],
      [141, 1000, 518_481],
      [124, 2000, 291_646],
      [124, 2000, 324_051],
      [124, 2000, 356_456],
      [124, 2000, 388_861],
      [128, 2000, 421_266],
      [132, 2000, 453_671],
      [137, 2000, 486_076],
      [141, 2000, 518_481],
      [124, 3000, 291_646],
      [124, 3000, 324_051],
      [124, 3000, 356_456],
      [124, 3000, 388_861],
      [128, 3000, 421_266],
      [133, 3000, 453_671],
      [137, 3000, 486_076],
      [141, 3000, 518_481],
      [123, 4000, 291_646],
      [123, 4000, 324_051],
      [123, 4000, 356_456],
      [123, 4000, 388_861],
      [128, 4000, 421_266],
      [133, 4000, 453_671],
      [137, 4000, 486_076],
      [141, 4000, 518_481],
      [123, 5000, 291_646],
      [123, 5000, 324_051],
      [123, 5000, 356_456],
      [123, 5000, 388_861],
      [128, 5000, 421_266],
      [133, 5000, 453_671],
      [137, 5000, 486_076],
      [142, 5000, 518_481],
      [122, 6000, 291_646],
      [122, 6000, 324_051],
      [122, 6000, 356_456],
      [123, 6000, 388_861],
      [128, 6000, 421_266],
      [133, 6000, 453_671],
      [137, 6000, 486_076],
      [142, 6000, 518_481],
      [121, 7000, 291_646],
      [121, 7000, 324_051],
      [121, 7000, 356_456],
      [123, 7000, 388_861],
      [128, 7000, 421_266],
      [133, 7000, 453_671],
      [138, 7000, 486_076],
      [142, 7000, 518_481],
      [119, 8000, 291_646],
      [119, 8000, 324_051],
      [119, 8000, 356_456],
      [123, 8000, 388_861],
      [128, 8000, 421_266],
      [133, 8000, 453_671],
      [138, 8000, 486_076],
      [142, 8000, 518_481],
      [118, 9000, 291_646],
      [118, 9000, 324_051],
      [118, 9000, 356_456],
      [123, 9000, 388_861],
      [128, 9000, 421_266],
      [133, 9000, 453_671],
      [138, 9000, 486_076],
      [142, 9000, 518_481],
      [116, 10000, 291_646],
      [116, 10000, 324_051],
      [118, 10000, 356_456],
      [123, 10000, 388_861],
      [128, 10000, 421_266],
      [133, 10000, 453_671],
      [138, 10000, 486_076],
      [142, 10000, 518_481],
      [114, 11000, 291_646],
      [114, 11000, 324_051],
      [118, 11000, 356_456],
      [123, 11000, 60_00],
      [128, 11000, 421_266],
      [133, 11000, 453_671],
      [138, 11000, 486_076],
      [142, 11000, 518_481],
      [112, 12000, 291_646],
      [113, 12000, 324_051],
      [118, 12000, 356_456],
      [123, 12000, 388_861],
      [128, 12000, 421_266],
      [133, 12000, 453_671],
      [138, 12000, 486_076],
      [143, 12000, 518_481],
      [110, 13000, 291_646],
      [113, 13000, 324_051],
      [118, 13000, 356_456],
      [123, 13000, 388_861],
      [128, 13000, 421_266],
      [133, 13000, 453_671],
      [138, 13000, 486_076],
      [143, 13000, 518_481],
      [108, 14100, 291_646],
      [113, 14100, 324_051],
      [118, 14100, 356_456],
      [123, 14100, 388_861],
      [128, 14100, 421_266],
      [134, 14100, 453_671],
      [139, 14100, 486_076],
      [143, 14100, 518_481],
      [107, 15100, 291_646],
      [113, 15100, 324_051],
      [118, 15100, 356_456],
      [123, 15100, 388_861],
      [129, 15100, 421_266],
      [134, 15100, 453_671],
      [139, 15100, 486_076],
      [143, 15100, 518_481],
    ]),
  };

  private static readonly v2RunwayVmcgBaseFactors: Record<number, [number, number]> = {
    1: [0.920413, 77.3469],
    2: [0.87805, 75.1346],
    3: [0.96131, 65.525],
  };

  private static readonly v2RunwayVmcgAltFactors: Record<number, [number, number]> = {
    1: [0.00002333, -0.00144],
    2: [0.00001713, -0.001057],
    3: [0.00001081, -0.0006236],
  };

  private static readonly vRRunwayVmcgBaseFactors: Record<number, [number, number]> = {
    1: [0.83076, 81.086],
    2: [0.728085, 84.111],
    3: [0.742761, 78.721],
  };

  private static readonly vRRunwayVmcgRunwayFactors: Record<number, [number, number, number]> = {
    1: [2280, 0.0001718, -0.01585],
    2: [2280, 0.0003239, -0.027272],
    3: [1900, 0.00057992, -0.045646],
  };

  private static readonly vRRunwayVmcgAltFactors: Record<number, [number, number]> = {
    1: [0.000029048, -0.001958],
    2: [0.000035557, -0.0025644],
    3: [1.02964e-5, -0.000545643],
  };

  private static readonly vRRunwayVmcgSlopeFactors: Record<number, number> = {
    1: 0.000887,
    2: 0.000887,
    3: 0.000887,
  };

  private static readonly vRRunwayVmcgHeadwindFactors: Record<number, [number, number]> = {
    1: [0, 0],
    2: [-0.027617, 2.1252],
    3: [0.003355, -0.263036],
  };

  private static readonly vRRunwayVmcgTailwindFactors: Record<number, [number, number]> = {
    1: [-0.008052, 0.6599],
    2: [-0.010709, 0.90273],
    3: [-0.027796, 2.107178],
  };

  private static readonly v1RunwayVmcgBaseFactors: Record<number, [number, number]> = {
    1: [0.4259042, 106.763],
    2: [0.398826, 106.337],
    3: [0.469648, 95.776],
  };

  private static readonly v1RunwayVmcgRunwayFactors: Record<number, [number, number, number]> = {
    1: [2280, 0.0003156, -0.03189],
    2: [2280, 0.0004396, -0.041238],
    3: [1900, 0.00144, -0.112592],
  };

  private static readonly v1RunwayVmcgAltFactors: Record<number, [number, number]> = {
    1: [0.00003416, -0.0028035],
    2: [0.00004354, -0.0035876],
    3: [0.0000847, -0.006666],
  };

  private static readonly v1RunwayVmcgSlopeFactors: Record<number, number> = {
    1: 0.000887,
    2: 0.000887,
    3: 0.000887,
  };

  private static readonly v1RunwayVmcgHeadwindFactors: Record<number, [number, number]> = {
    1: [0.00526, -0.2105],
    2: [0.00974, -0.53],
    3: [0.002333, -0.079528],
  };

  private static readonly v1RunwayVmcgTailwindFactors: Record<number, [number, number]> = {
    1: [-0.009243, 1.108],
    2: [-0.008207, 1.07],
    3: [-0.043516, 3.423],
  };

  private static readonly v2SecondSegBrakeThresholds: Record<number, [number, number]> = {
    1: [-0.009368, 186.79],
    2: [0.02346, 68.33],
    3: [0.022112, 83.141],
  };

  private static readonly v2SecondSegBrakeBaseTable1: Record<number, [number, number]> = {
    1: [0.72637, 101.077],
    2: [0.74005, 97.073],
    3: [0.3746, 130.078],
  };

  private static readonly v2SecondSegBrakeBaseTable2: Record<number, [number, number]> = {
    1: [0.63964, 102.127],
    2: [0.692636, 92.9863],
    3: [0.859926, 82.4377],
  };

  private static readonly v2SecondSegBrakeRunwayTable1: Record<number, [number, number]> = {
    1: [3180, -0.015997],
    2: [3180, -0.014862],
    3: [3180, -0.019296],
  };

  private static readonly v2SecondSegBrakeRunwayTable2: Record<number, [number, number]> = {
    1: [3180, -0.003612],
    2: [3180, -0.007],
    3: [3180, -0.013],
  };

  private static readonly v2SecondSegBrakeAltFactors: Record<number, [number, number, number, number]> = {
    1: [-0.00000924, -0.00075879, 0.000546, -1.075],
    2: [-0.00000387, -0.0009333, 0.000546, -1.075],
    3: [0.000034, -0.004043, 0.000468, -0.778471],
  };

  private static readonly v2SecondSegBrakeSlopeFactors: Record<number, [number, number]> = {
    1: [0.0000571, -0.008306],
    2: [0.0000286, -0.00415],
    3: [0.000001, -0.000556],
  };

  private static readonly v2SecondSegBrakeHeadwindFactors: Record<number, number> = {
    1: 0.2,
    2: 0.2,
    3: 0.2,
  };

  private static readonly v2SecondSegBrakeTailwindFactors: Record<number, number> = {
    1: 0.65,
    2: 0.5,
    3: 0.7,
  };

  private static readonly vRSecondSegBrakeBaseTable1: Record<number, [number, number]> = {
    1: [0.701509, 102.667],
    2: [0.696402, 100.226],
    3: [0.381534, 129.61],
  };

  private static readonly vRSecondSegBrakeBaseTable2: Record<number, [number, number]> = {
    1: [0.573107, 105.783],
    2: [0.932193, 115.336],
    3: [0.572407, 105.428],
  };

  private static readonly vRSecondSegBrakeRunwayTable1: Record<number, [number, number, number]> = {
    1: [3180, -0.000181, -0.005195],
    2: [3180, -0.000225, -0.000596],
    3: [3180, 0.000054, -0.024442],
  };

  private static readonly vRSecondSegBrakeRunwayTable2: Record<number, [number, number, number]> = {
    1: [3180, 0.004582, -0.395175],
    2: [3180, 0.000351, -0.03216],
    3: [3180, -0.000263, 0.014135],
  };

  private static readonly vRSecondSegBrakeAltTable1: Record<number, [number, number, number, number]> = {
    1: [-0.000034, 0.001018, 0.000154, 0.415385],
    2: [-0.00001, -0.000253, 0.000328, -0.24493],
    3: [0.000017, -0.003017, 0.000398, -0.5117],
  };

  private static readonly vRSecondSegBrakeAltTable2: Record<number, [number, number, number, number]> = {
    1: [0.000574, -0.047508, 0.000154, 0.415385],
    2: [0.000253, -0.019907, 0.000328, -0.24493],
    3: [0.000247, -0.019502, 0.000398, -0.5117],
  };

  private static readonly vRSecondSegBrakeSlopeFactors: Record<number, [number, number]> = {
    1: [0.000293, -0.023877],
    2: [0.000309, -0.025884],
    3: [0.000049, -0.005035],
  };

  private static readonly vRSecondSegBrakeHeadwindFactors: Record<number, [number, number]> = {
    1: [0.00668, -0.30215],
    2: [0.015247, -0.946949],
    3: [0.028496, -1.808403],
  };

  private static readonly vRSecondSegBrakeTailwindFactors: Record<number, [number, number]> = {
    1: [0.014683, -0.347428],
    2: [0.019024, -0.725293],
    3: [-0.002393, 0.994507],
  };

  private static readonly v1SecondSegBrakeBaseTable1: Record<number, [number, number]> = {
    1: [0.580888, 111.076],
    2: [0.663598, 102.54],
    3: [0.112254, 147.272],
  };

  private static readonly v1SecondSegBrakeBaseTable2: Record<number, [number, number]> = {
    1: [0.460256, 104.849],
    2: [0.583566, 84.342],
    3: [0.527615, 95.085],
  };

  private static readonly v1SecondSegBrakeRunwayTable1: Record<number, [number, number, number]> = {
    1: [3180, -0.000218, -0.003633],
    2: [3180, -0.000473, 0.015987],
    3: [3180, 0.000017, -0.022792],
  };

  private static readonly v1SecondSegBrakeRunwayTable2: Record<number, [number, number, number]> = {
    1: [3180, 0.005044, -0.418865],
    2: [3180, 0.00052, -0.024703],
    3: [3180, 0.000688, -0.048497],
  };

  private static readonly v1SecondSegBrakeAltTable1: Record<number, [number, number, number, number]> = {
    1: [-0.000084, 0.00393, 0.000231, 0.123077],
    2: [-0.0000086, -0.000333, 0.000172, 0.347893],
    3: [0.000159, -0.012122, 0.000382, -0.452242],
  };

  private static readonly v1SecondSegBrakeAltTable2: Record<number, [number, number, number, number]> = {
    1: [0.000957, -0.077197, 0.000231, 0.123077],
    2: [0.000354, -0.025738, 0.000172, 0.347893],
    3: [0.000927, -0.072365, 0.000382, -0.452242],
  };

  private static readonly v1SecondSegBrakeSlopeFactors: Record<number, [number, number]> = {
    1: [0.00003, -0.001069],
    2: [0.00003, -0.001069],
    3: [0.0000431, -0.003239],
  };

  private static readonly v1SecondSegBrakeHeadwindFactors: Record<number, [number, number]> = {
    1: [0.019515, -1.23885],
    2: [0.019515, -1.23886],
    3: [0.065846, -4.365037],
  };

  private static readonly v1SecondSegBrakeTailwindFactors: Record<number, [number, number]> = {
    1: [0.032069, -1.44],
    2: [0.030147, -1.4286],
    3: [-0.001744, 1.0938],
  };

  /**
   * Factors to determine the temperature above which we're VMCG limited on the wet runway.
   * Maps headwind component to TVMCG factors.
   */
  private static readonly tvmcgFactors: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec2Math.create(0.06485, -99.47), -15],
      [Vec2Math.create(0.9895, -116.54), 0],
      [Vec2Math.create(0.13858, -171.15), 10],
    ]),
    2: new LerpVectorLookupTable([
      [Vec2Math.create(0.06573, -102.97), -15],
      [Vec2Math.create(0.10579, -132.96), 0],
      [Vec2Math.create(0.06575, -64.4), 10],
    ]),
    3: new LerpVectorLookupTable([
      [Vec2Math.create(0.07002, -106.62), -15],
      [Vec2Math.create(0.08804, -108.42), 0],
      [Vec2Math.create(0.07728, -82.08), 10],
    ]),
  };

  /**
   * Factors to determine the TOW adjustment on a wet runway when not VMCG limited.
   * Maps headwind component to adjustment factors.
   */
  private static readonly wetTowAdjustmentFactorsAtOrBelowTvmcg: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [VecNMath.create(0.05498, -126.98, 0.00903, -28.35), -15],
      [VecNMath.create(0.02391, -48.94, 0.00043, -1.64), 0],
      [VecNMath.create(0.01044, -21.53, 0.00022, -1.12), 10],
    ]),
    2: new LerpVectorLookupTable([
      [VecNMath.create(0.03856, -94.674, 0.00965, -30.09), -15],
      [VecNMath.create(0.02686, -48.63, -0.00011, -0.08), 0],
      [VecNMath.create(0.00057, -2.94, -0.00004, -0.13), 10],
    ]),
    3: new LerpVectorLookupTable([
      [VecNMath.create(0.01924, -57.58, 0, 0), -15],
      [VecNMath.create(0.02184, -44.91, -0.00019, 0.1), 0],
      [VecNMath.create(0.00057, -2.94, 0.00047, -1.6), 10],
    ]),
  };

  /**
   * Factors to determine the TOW adjustment on a wet runway when VMCG limited.
   * Maps headwind component to adjustment factors.
   */
  private static readonly wetTowAdjustmentFactorsAboveTvmcg: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [VecNMath.create(0.0197, -61.23, 0, 0), -15],
      [VecNMath.create(0.01887, -48.47, 0, 0), 0],
      [VecNMath.create(0.045, -86.32, 0, 0), 10],
    ]),
    2: new LerpVectorLookupTable([
      [VecNMath.create(0.01941, -60.02, 0, 0), -15],
      [VecNMath.create(0.02797, -61.99, 0, 0), 0],
      [VecNMath.create(0.03129, -63.61, 0, 0), 10],
    ]),
    3: new LerpVectorLookupTable([
      [VecNMath.create(0.01978, -61.43, 0, 0), -15],
      [VecNMath.create(0.02765, -61.88, 0, 0), 0],
      [VecNMath.create(0.03662, -72.45, 0, 0), 10],
    ]),
  };

  /**
   * Factors to determine the flex adjustment on a wet runway when not VMCG limited.
   * Maps headwind component to adjustment factors.
   */
  private static readonly wetFlexAdjustmentFactorsAtOrBelowTvmcg: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [VecNMath.create(0.07933, -190.57, 0.02074, -65.05), -15],
      [VecNMath.create(0.04331, -90.86, 0.00098, -3.88), 0],
      [VecNMath.create(0.0233, -48.8, 0.00072, -3.14), 10],
    ]),
    2: new LerpVectorLookupTable([
      [VecNMath.create(0.029, -89.9, 0.0099, -38.42), -15],
      [VecNMath.create(0.03845, -80.2, -1, 0), 0],
      [VecNMath.create(0.00167, -7.01, 0.000266, -1.61), 10],
    ]),
    3: new LerpVectorLookupTable([
      [VecNMath.create(0.03993, -94.09, 0, 0), -15],
      [VecNMath.create(0.03845, -80.2, -1, 0), 0],
      [VecNMath.create(0.00835, -18.34, -1, 0), 10],
    ]),
  };

  /**
   * Factors to determine the flex adjustment on a wet runway when VMCG limited.
   * Maps headwind component to adjustment factors.
   */
  private static readonly wetFlexAdjustmentFactorsAboveTvmcg: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [VecNMath.create(-0.03716, 31.85, 0.08618, -234.92), -15],
      [VecNMath.create(-0.05861, 51.01, 0.04322, -113.39), 0],
      [VecNMath.create(0.1012, -195.48, 0, 0), 10],
    ]),
    2: new LerpVectorLookupTable([
      [VecNMath.create(-0.0285, 19.43, 0.06951, -193.38), -15],
      [VecNMath.create(-0.04698, 37.58, 0.06438, -139.9), 0],
      [VecNMath.create(0.06159, -126.56, 0, 0), 10],
    ]),
    3: new LerpVectorLookupTable([
      [VecNMath.create(-0.0024, 4.25, -0.02118, 46.2), -15],
      [VecNMath.create(-0.02645, 9.81, 0.06116, -131.79), 0],
      [VecNMath.create(0.04841, -104.22, 0, 0), 10],
    ]),
  };

  /**
   * Factors to determine the V1 adjustment on a wet runway when not VMCG limited.
   * Maps headwind component to adjustment factors.
   */
  private static readonly wetV1AdjustmentFactorsAtOrBelowTvmcg: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [VecNMath.create(0.01428, -32.58, 0.00048, -2.03), -15],
      [VecNMath.create(-0.00786, 6.81, 0.00234, -14.23), 0],
      [VecNMath.create(-0.00246, -3.68, 0.00145, -11.32), 10],
    ]),
    2: new LerpVectorLookupTable([
      [VecNMath.create(-0.01563, 28.93, -0.00559, 6.36), -15],
      [VecNMath.create(-0.00474, 6.98, 0.0024, -13.2), 0],
      [VecNMath.create(0.00236, -11.92, 0, 0), 10],
    ]),
    3: new LerpVectorLookupTable([
      [VecNMath.create(-0.01018, 18.83, 0, 0), -15],
      [VecNMath.create(-0.00931, 10.01, 0.00017, -8.5), 0],
      [VecNMath.create(-0.00005, -7.98, 0, 0), 10],
    ]),
  };

  /**
   * Factors to determine the V1 adjustment on a wet runway not VMCG limited.
   * Maps headwind component to adjustment factors.
   */
  private static readonly wetV1AdjustmentFactorsAboveTvmcg: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [VecNMath.create(-0.00131, 0.91, -0.02013, 42.56), -15],
      [VecNMath.create(0.10383, -169.42, -0.00529, 6.73), 0],
      [VecNMath.create(-0.01594, 21.8, 0, 0), 10],
    ]),
    2: new LerpVectorLookupTable([
      [VecNMath.create(-0.00789, 15.29, 0, 0), -15],
      [VecNMath.create(-0.00971, 14, 0, 0), 0],
      [VecNMath.create(-0.00684, 8.43, 0, 0), 10],
    ]),
    3: new LerpVectorLookupTable([
      [VecNMath.create(-0.0024, 4.25, -0.02118, 46.2), -15],
      [VecNMath.create(-0.00727, 10.12, 0, 0), 0],
      [VecNMath.create(-0.00671, 8.65, -0.0333, 50.84), 10],
    ]),
  };

  /**
   * Factors to determine the Vr adjustment on a wet runway when not VMCG limited.
   * Maps headwind component to adjustment factors.
   */
  private static readonly wetVRAdjustmentFactorsAtOrBelowTvmcg: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [VecNMath.create(0.01428, -32.58, 0.00048, -2.03), -15],
      [VecNMath.create(0.00353, -7.19, 0.00022, -0.64), 0],
      [VecNMath.create(0.0022, -4.14, 0.00053, -1.54), 10],
    ]),
    2: new LerpVectorLookupTable([
      [VecNMath.create(0.00693, -16.96, -0.00559, 6.36), -15],
      [VecNMath.create(0.00864, -17.08, 0, 0), 0],
      [VecNMath.create(0, 0, 0, 0), 10],
    ]),
    3: new LerpVectorLookupTable([
      [VecNMath.create(0.00151, -6.16, 0, 0), -15],
      [VecNMath.create(-0.00557, -11.68, -0.0004, 0.54), 0],
      [VecNMath.create(-0.0001, -0.11, 0, 0), 10],
    ]),
  };

  /**
   * Factors to determine the V2 adjustment on a wet runway when not VMCG limited.
   * Maps headwind component to adjustment factors.
   */
  private static readonly wetV2AdjustmentFactorsAtOrBelowTvmcg: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [VecNMath.create(0.01936, -43.79, 0.000483, -2.03), -15],
      [VecNMath.create(0.00353, -7.19, 0.00022, -0.64), 0],
      [VecNMath.create(0.0022, -4.14, 0.00053, -1.54), 10],
    ]),
    2: new LerpVectorLookupTable([
      [VecNMath.create(0.01198, -28.31, 0, 0), -15],
      [VecNMath.create(0.00864, -17.08, 0, 0), 0],
      [VecNMath.create(0, 0, 0, 0), 10],
    ]),
    3: new LerpVectorLookupTable([
      [VecNMath.create(0.00246, -8.65, 0, 0), -15],
      [VecNMath.create(0.00114, -3.52, 0, 0), 0],
      [VecNMath.create(-0.0001, -0.11, 0, 0), 10],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with 6.3 mm/1/4" of water.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminated6mmWater: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [97_215, 2_500],
      [93_327, 3_000],
      [68_051, 3_500],
      [6_800, 4_000],
    ]),
    2: new LerpLookupTable([
      [112_770, 2_000],
      [112_770, 2_500],
      [100_456, 3_000],
    ]),
    3: new LerpLookupTable([
      [121_195, 1_750],
      [121_195, 2_000],
      [112_770, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with 6.3 mm/1/4" of water. */
  private static minCorrectedTowContaminated6mmWater: Record<number, number> = {
    1: 374_603,
    2: 368_122,
    3: 370_714,
  };

  /**
   * MTOW for runways contaminated with 6.3 mm/1/4" of water.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminated6mmWater: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 374_603],
      [343_494, 382_380],
      [393_397, 393_397],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 368_122],
      [382_380, 382_380],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 370_714],
      [362_937, 382_380],
      [387_565, 387_565],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with 6.3 mm/1/4" of water.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminated6mmWater: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 125, 127), 308_496],
      [Vec3Math.create(122, 127, 129), 317_570],
      [Vec3Math.create(122, 133, 135), 349_975],
      [Vec3Math.create(122, 139, 141), 382_380],
      [Vec3Math.create(122, 141, 143), 393_397],
      [Vec3Math.create(126, 145, 147), 414_785],
      [Vec3Math.create(132, 151, 153), 447_190],
      [Vec3Math.create(137, 156, 158), 479_595],
      [Vec3Math.create(142, 161, 163), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 141, 142), 382_380],
      [Vec3Math.create(127, 146, 147), 414_785],
      [Vec3Math.create(133, 152, 153), 447_190],
      [Vec3Math.create(139, 158, 159), 479_595],
      [Vec3Math.create(144, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(122, 128, 128), 317_570],
      [Vec3Math.create(122, 134, 134), 349_975],
      [Vec3Math.create(122, 140, 140), 382_380],
      [Vec3Math.create(122, 141, 141), 387_565],
      [Vec3Math.create(127, 146, 146), 414_785],
      [Vec3Math.create(133, 152, 152), 447_190],
      [Vec3Math.create(139, 158, 158), 479_595],
      [Vec3Math.create(144, 163, 163), 512_000],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with 12.7 mm/1/2" of water.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminated13mmWater: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [121_195, 2_500],
      [113_418, 3_000],
      [88_142, 3_500],
      [9_700, 4_000],
    ]),
    2: new LerpLookupTable([
      [136_101, 2_000],
      [134_805, 2_500],
      [119_899, 3_000],
    ]),
    3: new LerpLookupTable([
      [141_934, 1_750],
      [141_934, 2_000],
      [132_861, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with 12.7 mm/1/2" of water. */
  private static minCorrectedTowContaminated13mmWater: Record<number, number> = {
    1: 345_438,
    2: 345_438,
    3: 366_177,
  };

  /**
   * MTOW for runways contaminated with 12.7 mm/1/2" of water.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminated13mmWater: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 345_438],
      [330_532, 349_975],
      [355_159, 355_159],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 345_438],
      [330_532, 349_975],
      [354_511, 354_511],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 366_177],
      [382_380, 382_380],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with 12.7 mm/1/2" of water.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminated13mmWater: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 125, 127), 308_496],
      [Vec3Math.create(122, 127, 129), 317_570],
      [Vec3Math.create(122, 133, 135), 349_975],
      [Vec3Math.create(122, 134, 136), 355_159],
      [Vec3Math.create(127, 139, 141), 382_380],
      [Vec3Math.create(133, 145, 147), 414_785],
      [Vec3Math.create(132, 151, 153), 447_190],
      [Vec3Math.create(144, 156, 158), 479_595],
      [Vec3Math.create(149, 161, 163), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 135, 136), 354_511],
      [Vec3Math.create(128, 141, 142), 382_380],
      [Vec3Math.create(133, 146, 147), 414_785],
      [Vec3Math.create(139, 152, 153), 447_190],
      [Vec3Math.create(145, 158, 159), 479_595],
      [Vec3Math.create(150, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(122, 128, 128), 317_570],
      [Vec3Math.create(122, 134, 134), 349_975],
      [Vec3Math.create(122, 140, 140), 382_380],
      [Vec3Math.create(128, 146, 146), 414_785],
      [Vec3Math.create(134, 152, 152), 447_190],
      [Vec3Math.create(140, 158, 158), 479_595],
      [Vec3Math.create(145, 163, 163), 512_000],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with 6.3 mm/1/4" of slush.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminated6mmSlush: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [99_808, 2_500],
      [92_030, 3_000],
      [67_403, 3_500],
      [6_600, 4_000],
    ]),
    2: new LerpLookupTable([
      [116_010, 2_000],
      [115_362, 2_500],
      [100_456, 3_000],
    ]),
    3: new LerpLookupTable([
      [124_435, 1_750],
      [123_139, 2_000],
      [111_473, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with 6.3 mm/1/4" of slush. */
  private static minCorrectedTowContaminated6mmSlush: Record<number, number> = {
    1: 370_714,
    2: 364_233,
    3: 345_438,
  };

  /**
   * MTOW for runways contaminated with 6.3 mm/1/4" of slush.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminated6mmSlush: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 370_714],
      [362_937, 382_380],
      [387_565, 387_565],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 364_233],
      [362_937, 382_380],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 345_438],
      [330_532, 349_975],
      [355_159, 355_159],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with 6.3 mm/1/4" of slush.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminated6mmSlush: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 125, 127), 308_496],
      [Vec3Math.create(122, 127, 129), 317_570],
      [Vec3Math.create(122, 133, 135), 349_975],
      [Vec3Math.create(127, 139, 141), 382_380],
      [Vec3Math.create(127, 140, 142), 387_565],
      [Vec3Math.create(127, 145, 147), 414_785],
      [Vec3Math.create(133, 151, 153), 447_190],
      [Vec3Math.create(138, 156, 158), 479_595],
      [Vec3Math.create(143, 161, 163), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 140, 141), 377_843],
      [Vec3Math.create(123, 141, 142), 382_380],
      [Vec3Math.create(128, 146, 147), 414_785],
      [Vec3Math.create(134, 152, 153), 447_190],
      [Vec3Math.create(140, 158, 159), 479_595],
      [Vec3Math.create(145, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(122, 128, 128), 317_570],
      [Vec3Math.create(122, 134, 134), 349_975],
      [Vec3Math.create(122, 135, 135), 355_159],
      [Vec3Math.create(127, 140, 140), 382_380],
      [Vec3Math.create(133, 146, 146), 414_785],
      [Vec3Math.create(139, 152, 152), 447_190],
      [Vec3Math.create(145, 158, 158), 479_595],
      [Vec3Math.create(150, 163, 163), 512_000],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with 12.7 mm/1/2" of slush.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminated13mmSlush: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [125_732, 2_500],
      [115_362, 3_000],
      [88_790, 3_500],
      [10_000, 4_000],
    ]),
    2: new LerpLookupTable([
      [142_582, 2_000],
      [139_990, 2_500],
      [121_843, 3_000],
    ]),
    3: new LerpLookupTable([
      [117_954, 1_750],
      [146_471, 2_000],
      [135_453, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with 12.7 mm/1/2" of slush. */
  private static minCorrectedTowContaminated13mmSlush: Record<number, number> = {
    1: 340_901,
    2: 337_013,
    3: 337_013,
  };

  /**
   * MTOW for runways contaminated with 12.7 mm/1/2" of slush.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminated13mmSlush: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 340_901],
      [349_975, 349_975],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 337_013],
      [344_790, 344_790],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 337_013],
      [344_790, 344_790],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with 12.7 mm/1/2" of slush.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminated13mmSlush: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(128, 140, 141), 382_380],
      [Vec3Math.create(134, 146, 147), 414_785],
      [Vec3Math.create(140, 152, 153), 447_190],
      [Vec3Math.create(145, 157, 158), 479_595],
      [Vec3Math.create(150, 162, 163), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 133, 134), 344_790],
      [Vec3Math.create(123, 134, 135), 349_975],
      [Vec3Math.create(130, 141, 142), 382_380],
      [Vec3Math.create(135, 146, 147), 414_785],
      [Vec3Math.create(141, 152, 153), 447_190],
      [Vec3Math.create(147, 158, 159), 479_595],
      [Vec3Math.create(152, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(122, 128, 128), 317_570],
      [Vec3Math.create(122, 133, 133), 344_790],
      [Vec3Math.create(123, 134, 134), 349_975],
      [Vec3Math.create(129, 140, 140), 382_380],
      [Vec3Math.create(135, 146, 146), 414_785],
      [Vec3Math.create(141, 152, 152), 447_190],
      [Vec3Math.create(147, 158, 158), 479_595],
      [Vec3Math.create(152, 163, 163), 512_000],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with compacted snow.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminatedCompactedSnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [66_106, 2_500],
      [9_000, 3_000],
      [4_700, 3_500],
      [3_000, 4_000],
    ]),
    2: new LerpLookupTable([
      [86_197, 2_000],
      [82_309, 2_500],
      [66_754, 3_000],
    ]),
    3: new LerpLookupTable([
      [97_863, 1_750],
      [95_919, 2_000],
      [82_309, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with compacted snow. */
  private static minCorrectedTowContaminatedCompactedSnow: Record<number, number> = {
    1: 370_714,
    2: 364_233,
    3: 366_177,
  };

  /**
   * MTOW for runways contaminated with compacted snow.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminatedCompactedSnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 370_714],
      [362_937, 382_380],
      [387_565, 387_565],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 364_233],
      [377_843, 377_843],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 366_177],
      [382_380, 382_380],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with compacted snow.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminatedCompactedSnow: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 140, 141), 382_380],
      [Vec3Math.create(122, 141, 142), 387_565],
      [Vec3Math.create(127, 146, 147), 414_785],
      [Vec3Math.create(133, 152, 153), 447_190],
      [Vec3Math.create(138, 157, 158), 479_595],
      [Vec3Math.create(143, 162, 163), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 140, 141), 377_843],
      [Vec3Math.create(123, 141, 142), 382_380],
      [Vec3Math.create(128, 146, 147), 414_785],
      [Vec3Math.create(134, 152, 153), 447_190],
      [Vec3Math.create(140, 158, 159), 479_595],
      [Vec3Math.create(145, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(122, 128, 128), 317_570],
      [Vec3Math.create(122, 134, 134), 349_975],
      [Vec3Math.create(122, 140, 140), 382_380],
      [Vec3Math.create(128, 146, 146), 414_785],
      [Vec3Math.create(134, 152, 152), 447_190],
      [Vec3Math.create(140, 158, 158), 479_595],
      [Vec3Math.create(145, 163, 163), 512_000],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with 5 mm/1/5" of wet snow.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminated5mmWetSnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [73_884, 2_500],
      [68_051, 3_000],
      [6_000, 3_500],
      [3_000, 4_000],
    ]),
    2: new LerpLookupTable([
      [97_215, 2_000],
      [92_030, 2_500],
      [77_124, 3_000],
    ]),
    3: new LerpLookupTable([
      [113_418, 1_750],
      [110_825, 2_000],
      [92_678, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with 5 mm/1/5" of wet snow. */
  private static minCorrectedTowContaminated5mmWetSnow: Record<number, number> = {
    1: 366_177,
    2: 372_010,
    3: 374_603,
  };

  /**
   * MTOW for runways contaminated with 5 mm/1/5" of wet snow.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminated5mmWetSnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 366_177],
      [382_380, 382_380],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 372_010],
      [362_937, 382_380],
      [388_861, 388_861],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 374_603],
      [343_494, 382_380],
      [393_397, 393_397],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with 5 mm/1/5" of wet snow.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminated5mmWetSnow: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 140, 141), 382_380],
      [Vec3Math.create(128, 146, 147), 414_785],
      [Vec3Math.create(134, 152, 153), 447_190],
      [Vec3Math.create(139, 157, 158), 479_595],
      [Vec3Math.create(144, 162, 163), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 141, 142), 382_380],
      [Vec3Math.create(122, 142, 143), 388_861],
      [Vec3Math.create(126, 146, 147), 414_785],
      [Vec3Math.create(132, 152, 153), 447_190],
      [Vec3Math.create(138, 158, 159), 479_595],
      [Vec3Math.create(143, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(122, 128, 128), 317_570],
      [Vec3Math.create(122, 134, 134), 349_975],
      [Vec3Math.create(122, 140, 140), 382_380],
      [Vec3Math.create(122, 142, 142), 393_397],
      [Vec3Math.create(126, 146, 146), 414_785],
      [Vec3Math.create(132, 152, 152), 447_190],
      [Vec3Math.create(138, 158, 158), 479_595],
      [Vec3Math.create(143, 163, 163), 512_000],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with 15 mm/3/5" of wet snow.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminated15mmWetSnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [105_641, 2_500],
      [96_567, 3_000],
      [70_643, 3_500],
      [7_100, 4_000],
    ]),
    2: new LerpLookupTable([
      [123_139, 2_000],
      [121_195, 2_500],
      [104_344, 3_000],
    ]),
    3: new LerpLookupTable([
      [130_916, 1_750],
      [129_620, 2_000],
      [117_306, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with 15 mm/3/5" of wet snow. */
  private static minCorrectedTowContaminated15mmWetSnow: Record<number, number> = {
    1: 349_327,
    2: 345_438,
    3: 345_438,
  };

  /**
   * MTOW for runways contaminated with 15 mm/3/5" of wet snow.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminated15mmWetSnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 349_327],
      [311_089, 349_975],
      [360_992, 360_992],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 345_438],
      [330_532, 349_975],
      [354_511, 354_511],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 345_438],
      [330_532, 349_975],
      [355_159, 355_159],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with 15 mm/3/5" of wet snow.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminated15mmWetSnow: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 136, 137), 360_992],
      [Vec3Math.create(126, 140, 141), 382_380],
      [Vec3Math.create(132, 146, 147), 414_785],
      [Vec3Math.create(138, 152, 153), 447_190],
      [Vec3Math.create(143, 157, 158), 479_595],
      [Vec3Math.create(148, 162, 163), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 135, 136), 354_511],
      [Vec3Math.create(128, 141, 142), 382_380],
      [Vec3Math.create(133, 146, 147), 414_785],
      [Vec3Math.create(139, 152, 153), 447_190],
      [Vec3Math.create(145, 158, 159), 479_595],
      [Vec3Math.create(150, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(122, 128, 128), 317_570],
      [Vec3Math.create(122, 134, 134), 349_975],
      [Vec3Math.create(122, 135, 135), 355_159],
      [Vec3Math.create(127, 140, 140), 382_380],
      [Vec3Math.create(133, 146, 146), 414_785],
      [Vec3Math.create(139, 152, 152), 447_190],
      [Vec3Math.create(145, 158, 158), 479_595],
      [Vec3Math.create(150, 163, 163), 512_000],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with 30 mm/6/5" of wet snow.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminated30mmWetSnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [149_063, 2_500],
      [136_749, 3_000],
      [124_435, 3_500],
      [124_435, 4_000],
    ]),
    2: new LerpLookupTable([
      [160_081, 2_000],
      [158_137, 2_500],
      [145_175, 3_000],
    ]),
    3: new LerpLookupTable([
      [163_322, 1_750],
      [162_025, 2_000],
      [153_600, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with 30 mm/6/5" of wet snow. */
  private static minCorrectedTowContaminated30mmWetSnow: Record<number, number> = {
    1: 311_737,
    2: 308_496,
    3: 308_496,
  };

  /**
   * MTOW for runways contaminated with 30 mm/6/5" of wet snow.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminated30mmWetSnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 311_737],
      [313_033, 313_033],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 308_496],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 308_496],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with 30 mm/6/5" of wet snow.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminated30mmWetSnow: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 127, 128), 313_033],
      [Vec3Math.create(123, 128, 129), 317_570],
      [Vec3Math.create(130, 134, 135), 349_975],
      [Vec3Math.create(137, 141, 142), 382_380],
      [Vec3Math.create(142, 146, 147), 414_785],
      [Vec3Math.create(148, 152, 153), 447_190],
      [Vec3Math.create(154, 158, 159), 479_595],
      [Vec3Math.create(159, 163, 164), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(124, 128, 129), 317_570],
      [Vec3Math.create(130, 134, 135), 349_975],
      [Vec3Math.create(137, 141, 142), 382_380],
      [Vec3Math.create(142, 146, 147), 414_785],
      [Vec3Math.create(148, 152, 153), 447_190],
      [Vec3Math.create(154, 158, 159), 479_595],
      [Vec3Math.create(159, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(124, 128, 128), 317_570],
      [Vec3Math.create(130, 134, 134), 349_975],
      [Vec3Math.create(136, 140, 140), 382_380],
      [Vec3Math.create(142, 146, 146), 414_785],
      [Vec3Math.create(148, 152, 152), 447_190],
      [Vec3Math.create(154, 158, 158), 479_595],
      [Vec3Math.create(159, 163, 163), 512_000],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with 10 mm/2/5" of dry snow.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminated10mmDrySnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [73_884, 2_500],
      [68_051, 3_000],
      [6_000, 3_500],
      [3_000, 4_000],
    ]),
    2: new LerpLookupTable([
      [97_215, 2_000],
      [92_030, 2_500],
      [76_476, 3_000],
    ]),
    3: new LerpLookupTable([
      [113_418, 1_750],
      [110_825, 2_000],
      [92_678, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with 10 mm/2/5" of dry snow. */
  private static minCorrectedTowContaminated10mmDrySnow: Record<number, number> = {
    1: 366_177,
    2: 372_010,
    3: 374_603,
  };

  /**
   * MTOW for runways contaminated with 10 mm/2/5" of dry snow.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminated10mmDrySnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 366_177],
      [382_380, 382_380],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 372_010],
      [362_937, 382_380],
      [388_861, 388_861],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 374_603],
      [343_494, 382_380],
      [393_397, 393_397],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with 10 mm/2/5" of dry snow.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminated10mmDrySnow: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 140, 141), 382_380],
      [Vec3Math.create(128, 146, 147), 414_785],
      [Vec3Math.create(134, 152, 153), 447_190],
      [Vec3Math.create(139, 157, 158), 479_595],
      [Vec3Math.create(144, 162, 163), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(122, 134, 135), 349_975],
      [Vec3Math.create(122, 141, 142), 382_380],
      [Vec3Math.create(122, 142, 143), 388_861],
      [Vec3Math.create(126, 146, 147), 414_785],
      [Vec3Math.create(132, 152, 153), 447_190],
      [Vec3Math.create(138, 158, 159), 479_595],
      [Vec3Math.create(143, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(122, 128, 128), 317_570],
      [Vec3Math.create(122, 134, 134), 349_975],
      [Vec3Math.create(122, 140, 140), 382_380],
      [Vec3Math.create(122, 142, 142), 393_397],
      [Vec3Math.create(126, 146, 146), 414_785],
      [Vec3Math.create(132, 152, 152), 447_190],
      [Vec3Math.create(138, 158, 158), 479_595],
      [Vec3Math.create(143, 163, 163), 512_000],
    ]),
  };

  /**
   * Weight Correction for runways contaminated with 100 mm/4" of dry snow.
   * Maps runway length in metres to weight correction in kg.
   * Note: no clearway figures used, as MSFS can't tell us if there's a clearway.
   */
  private static readonly weightCorrectionContaminated100mmDrySnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [125_084, 2_500],
      [127_028, 3_000],
      [113_418, 3_500],
      [102_400, 4_000],
    ]),
    2: new LerpLookupTable([
      [136_749, 2_000],
      [142_582, 2_500],
      [138_694, 3_000],
    ]),
    3: new LerpLookupTable([
      [143_230, 1_750],
      [144_527, 2_000],
      [141_286, 2_500],
    ]),
  };

  /** Minimum takeoff weight for each config on runways contaminated with 100 mm/4" of dry snow. */
  private static minCorrectedTowContaminated100mmDrySnow: Record<number, number> = {
    1: 315_625,
    2: 313_033,
    3: 315_625,
  };

  /**
   * MTOW for runways contaminated with 100 mm/4" of dry snow.
   * Maps corrected weight in kg to MTOW in kg.
   */
  private static readonly mtowContaminated100mmDrySnow: Record<number, LerpLookupTable> = {
    1: new LerpLookupTable([
      [308_496, 315_625],
      [317_570, 317_570],
      [512_000, 512_000],
    ]),
    2: new LerpLookupTable([
      [308_496, 311_737],
      [313_033, 313_033],
      [512_000, 512_000],
    ]),
    3: new LerpLookupTable([
      [308_496, 315_625],
      [317_570, 317_570],
      [512_000, 512_000],
    ]),
  };

  /**
   * V-Speeds for runways contaminated with 100 mm/4" of dry snow.
   * Maps actual takeoff weight in kg to v1, vr, v2 in knots.
   */
  private static readonly vSpeedsContaminated100mmDrySnow: Record<number, LerpVectorLookupTable> = {
    1: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 128, 129), 317_570],
      [Vec3Math.create(128, 134, 135), 349_975],
      [Vec3Math.create(134, 140, 141), 382_380],
      [Vec3Math.create(140, 146, 147), 414_785],
      [Vec3Math.create(146, 152, 153), 447_190],
      [Vec3Math.create(151, 157, 158), 479_595],
      [Vec3Math.create(156, 162, 163), 512_000],
    ]),
    2: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 127), 308_496],
      [Vec3Math.create(122, 127, 128), 313_033],
      [Vec3Math.create(123, 128, 129), 317_570],
      [Vec3Math.create(129, 134, 135), 349_975],
      [Vec3Math.create(136, 141, 142), 382_380],
      [Vec3Math.create(141, 146, 147), 414_785],
      [Vec3Math.create(147, 152, 153), 447_190],
      [Vec3Math.create(153, 158, 159), 479_595],
      [Vec3Math.create(158, 163, 164), 512_000],
    ]),
    3: new LerpVectorLookupTable([
      [Vec3Math.create(122, 126, 126), 308_496],
      [Vec3Math.create(122, 128, 128), 317_570],
      [Vec3Math.create(128, 134, 134), 349_975],
      [Vec3Math.create(134, 140, 140), 382_380],
      [Vec3Math.create(140, 146, 146), 414_785],
      [Vec3Math.create(146, 152, 152), 447_190],
      [Vec3Math.create(152, 158, 158), 479_595],
      [Vec3Math.create(157, 163, 163), 512_000],
    ]),
  };

  public getCrosswindLimit(runwayCondition: RunwayCondition, oat: number): number {
    switch (runwayCondition) {
      case RunwayCondition.Dry:
      case RunwayCondition.Wet:
        return 35;
      case RunwayCondition.ContaminatedCompactedSnow:
        return oat <= -15 ? 29 : 25;
      case RunwayCondition.Contaminated10mmDrySnow:
      case RunwayCondition.Contaminated100mmDrySnow:
      case RunwayCondition.Contaminated5mmWetSnow:
      case RunwayCondition.Contaminated15mmWetSnow:
      case RunwayCondition.Contaminated30mmWetSnow:
        return 25;
      case RunwayCondition.Contaminated6mmWater:
      case RunwayCondition.Contaminated13mmWater:
      case RunwayCondition.Contaminated6mmSlush:
      case RunwayCondition.Contaminated13mmSlush:
        return 20;
    }
  }

  private checkInputs(inputs: TakeoffPerformanceInputs, params: TakeoffPerformanceParameters): TakeoffPerfomanceError {
    if (inputs.conf !== 1 && inputs.conf !== 2 && inputs.conf !== 3) {
      return TakeoffPerfomanceError.InvalidData;
    }
    if (inputs.tow > this.structuralMtow) {
      return TakeoffPerfomanceError.StructuralMtow;
    }
    if (params.pressureAlt > this.maxPressureAlt) {
      return TakeoffPerfomanceError.MaximumPressureAlt;
    }
    if (inputs.oat > params.tMax) {
      return TakeoffPerfomanceError.MaximumTemperature;
    }
    if (inputs.tow < this.oew) {
      return TakeoffPerfomanceError.OperatingEmptyWeight;
    }
    if (inputs.cg !== undefined && !this.isCgWithinLimits(inputs.cg, inputs.tow)) {
      return TakeoffPerfomanceError.CgOutOfLimits;
    }
    if (inputs.wind < -this.maxTailwind) {
      return TakeoffPerfomanceError.MaximumTailwind;
    }
    if (Math.abs(inputs.slope) > 2) {
      return TakeoffPerfomanceError.MaximumRunwaySlope;
    }

    return TakeoffPerfomanceError.None;
  }

  private isContaminated(runwayCondition: RunwayCondition): boolean {
    return runwayCondition !== RunwayCondition.Dry && runwayCondition !== RunwayCondition.Wet;
  }

  /** @inheritdoc */
  public calculateTakeoffPerformance(
    tow: number,
    forwardCg: boolean,
    conf: number,
    tora: number,
    slope: number,
    lineupAngle: LineupAngle,
    wind: number,
    elevation: number,
    qnh: number,
    oat: number,
    antiIce: TakeoffAntiIceSetting,
    packs: boolean,
    forceToga: boolean,
    runwayCondition: RunwayCondition,
    cg?: number,
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    out?: Partial<TakeoffPerformanceResult>,
  ): TakeoffPerformanceResult {
    const result: Partial<TakeoffPerformanceResult> = {}; // out === undefined ? {} : out;
    result.inputs = {
      tow,
      forwardCg,
      cg,
      conf,
      tora,
      slope,
      lineupAngle,
      wind,
      elevation,
      qnh,
      oat,
      antiIce,
      packs,
      forceToga,
      runwayCondition,
    };

    const isaTemp = this.calculateIsaTemp(elevation);
    const tRef = this.calculateTref(elevation);
    const pressureAlt = this.calculatePressureAltitude(elevation, qnh);
    const tMax = this.calculateTmax(pressureAlt);
    const tFlexMax = this.calculateTflexMax(isaTemp);
    // headwind credit is limited to 45 knots
    const headwind = Math.min(this.maxHeadwind, wind);

    result.params = {
      adjustedTora: tora - (A380482xTakeoffPerformanceCalculator.lineUpDistances[lineupAngle] ?? 0),
      pressureAlt,
      isaTemp,
      tRef,
      tMax,
      tFlexMax,
      headwind,
    };

    result.error = this.checkInputs(result.inputs, result.params);

    if (result.error === TakeoffPerfomanceError.None) {
      result.limits = {
        [LimitingFactor.Runway]: this.calculateWeightLimits(LimitingFactor.Runway, result),
        [LimitingFactor.SecondSegment]: this.calculateWeightLimits(LimitingFactor.SecondSegment, result),
        [LimitingFactor.BrakeEnergy]: this.calculateWeightLimits(LimitingFactor.BrakeEnergy, result),
        [LimitingFactor.Vmcg]: this.calculateWeightLimits(LimitingFactor.Vmcg, result),
      };

      result.oatLimitingFactor = this.getLimitingFactor('oatLimit', result);
      result.tRefLimitingFactor = this.getLimitingFactor('tRefLimit', result);
      result.tMaxLimitingFactor = this.getLimitingFactor('tMaxLimit', result);
      result.tFlexMaxLimitingFactor = this.getLimitingFactor('tFlexMaxLimit', result);

      const dryMtow = result.limits[result.tRefLimitingFactor].oatLimit!;
      result.tvmcg = this.calculateTvmcg(result.inputs, result.params);

      let mtow: number;
      if (runwayCondition === RunwayCondition.Dry) {
        mtow = dryMtow;
      } else if (runwayCondition === RunwayCondition.Wet) {
        const factors: ReadonlyFloat64Array = (
          result.inputs.oat > result.tvmcg
            ? A380482xTakeoffPerformanceCalculator.wetTowAdjustmentFactorsAboveTvmcg
            : A380482xTakeoffPerformanceCalculator.wetTowAdjustmentFactorsAtOrBelowTvmcg
        )[result.inputs.conf].get(A380482xTakeoffPerformanceCalculator.vec4Cache, result.params.headwind);

        const lengthAltCoef = result.params.adjustedTora - result.params.pressureAlt / 20;
        const wetMtowAdjustment = Math.min(
          0,
          factors[0] * lengthAltCoef + factors[1],
          factors[2] * lengthAltCoef + factors[3],
        );
        mtow = dryMtow - wetMtowAdjustment;
      } else {
        let correctionFactors: Record<number, LerpLookupTable>;
        let mtowFactors: Record<number, LerpLookupTable>;
        let minCorrectedWeight: Record<number, number>;
        switch (runwayCondition) {
          case RunwayCondition.Contaminated6mmWater:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminated6mmWater;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminated6mmWater;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminated6mmWater;
            break;
          case RunwayCondition.Contaminated13mmWater:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminated13mmWater;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminated13mmWater;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminated13mmWater;
            break;
          case RunwayCondition.Contaminated6mmSlush:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminated6mmSlush;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminated6mmSlush;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminated6mmSlush;
            break;
          case RunwayCondition.Contaminated13mmSlush:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminated13mmSlush;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminated13mmSlush;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminated13mmSlush;
            break;
          case RunwayCondition.ContaminatedCompactedSnow:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminatedCompactedSnow;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminatedCompactedSnow;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminatedCompactedSnow;
            break;
          case RunwayCondition.Contaminated5mmWetSnow:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminated5mmWetSnow;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminated5mmWetSnow;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminated5mmWetSnow;
            break;
          case RunwayCondition.Contaminated15mmWetSnow:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminated15mmWetSnow;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminated15mmWetSnow;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminated15mmWetSnow;
            break;
          case RunwayCondition.Contaminated30mmWetSnow:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminated30mmWetSnow;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminated30mmWetSnow;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminated30mmWetSnow;
            break;
          case RunwayCondition.Contaminated10mmDrySnow:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminated10mmDrySnow;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminated10mmDrySnow;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminated10mmDrySnow;
            break;
          case RunwayCondition.Contaminated100mmDrySnow:
            correctionFactors = A380482xTakeoffPerformanceCalculator.weightCorrectionContaminated100mmDrySnow;
            mtowFactors = A380482xTakeoffPerformanceCalculator.mtowContaminated100mmDrySnow;
            minCorrectedWeight = A380482xTakeoffPerformanceCalculator.minCorrectedTowContaminated100mmDrySnow;
            break;
          default:
            throw new Error('Invalid runway condition');
        }

        const correctedWeight = dryMtow - correctionFactors[result.inputs.conf].get(result.params.adjustedTora);
        mtow = mtowFactors[result.inputs.conf].get(correctedWeight);

        const minimumTow = minCorrectedWeight[result.inputs.conf];
        if (correctedWeight < minimumTow) {
          result.error = TakeoffPerfomanceError.TooLight;
        }
      }
      result.mtow = mtow;

      const applyForwardCgWeightCorrection =
        forwardCg &&
        (result.oatLimitingFactor === LimitingFactor.Runway || result.oatLimitingFactor === LimitingFactor.Vmcg);
      const applyForwardCgSpeedCorrection = applyForwardCgWeightCorrection && mtow <= 473_114;

      if (applyForwardCgWeightCorrection) {
        const cgFactors = A380482xTakeoffPerformanceCalculator.cgFactors[conf];
        mtow += Math.max(0, cgFactors[0] * mtow + cgFactors[1]);
      }

      if (mtow >= tow) {
        result.flex = undefined;

        let needVSpeedCalculated = true;
        if (forceToga) {
          // find the speeds for a flex takeoff with 15 knot tailwind
          const tailwindResult = this.calculateTakeoffPerformance(
            tow,
            forwardCg,
            conf,
            tora,
            slope,
            lineupAngle,
            -15,
            elevation,
            qnh,
            oat,
            antiIce,
            packs,
            false,
            runwayCondition,
            cg,
            A380482xTakeoffPerformanceCalculator.resultCache,
          );

          if (tailwindResult.error === TakeoffPerfomanceError.None) {
            needVSpeedCalculated = false;
            result.v1 = tailwindResult.v1;
            result.vR = tailwindResult.vR;
            result.v2 = tailwindResult.v2;
            result.intermediateSpeeds = tailwindResult.intermediateSpeeds
              ? { ...tailwindResult.intermediateSpeeds }
              : undefined;
          } // else we use the higher speeds below...
        } else if (!this.isContaminated(result.inputs.runwayCondition)) {
          [result.flex, result.params.flexLimitingFactor] = this.calculateFlexTemp(result, result.tvmcg);
        }

        if (needVSpeedCalculated) {
          this.calculateVSpeeds(result, applyForwardCgSpeedCorrection, result.tvmcg);
        }
      } else {
        result.error = TakeoffPerfomanceError.TooHeavy;
      }
    }

    if (cg !== undefined) {
      result.stabTrim = this.calculateStabTrim(cg);
    } else {
      result.stabTrim = undefined;
    }

    return result as TakeoffPerformanceResult;
  }

  private calculateFlexTow(
    result: Partial<TakeoffPerformanceResult>,
    limitingFactor: LimitingFactor,
    limitingWeights: LimitWeight,
    temperature: number,
  ): number {
    if (!result.inputs || !result.params) {
      throw new Error('Invalid result object!');
    }

    switch (limitingFactor) {
      case LimitingFactor.Runway:
        return (
          limitingWeights.altLimit -
          this.calculateRunwayTempDelta(
            temperature,
            result.inputs.conf,
            result.params.tRef,
            result.params.tMax,
            result.params.tFlexMax,
            result.params.adjustedTora,
            result.params.pressureAlt,
            result.params.isaTemp,
          ) -
          this.calculateRunwayWindDelta(
            temperature,
            result.inputs.conf,
            result.params.isaTemp,
            result.params.tRef,
            result.params.tMax,
            result.params.tFlexMax,
            result.params.adjustedTora,
            result.params.headwind,
          )
        );
        break;
      case LimitingFactor.SecondSegment:
        return (
          limitingWeights.altLimit -
          this.calculateSecondSegmentTempDelta(
            temperature,
            result.inputs.conf,
            result.params.tRef,
            result.params.tMax,
            result.params.tFlexMax,
            result.params.adjustedTora,
            result.params.pressureAlt,
            result.params.isaTemp,
          ) -
          this.calculateSecondSegmentWindDelta(
            temperature,
            result.inputs.conf,
            result.params.isaTemp,
            result.params.tRef,
            result.params.tMax,
            result.params.tFlexMax,
            result.params.adjustedTora,
            result.params.headwind,
          )
        );
        break;
      case LimitingFactor.BrakeEnergy:
        return (
          limitingWeights.altLimit -
          this.calculateBrakeEnergyTempDelta(
            temperature,
            result.inputs.conf,
            result.params.tRef,
            result.params.tMax,
            result.params.tFlexMax,
            result.params.adjustedTora,
            result.params.pressureAlt,
            result.params.isaTemp,
          ) -
          this.calculateBrakeEnergyWindDelta(
            temperature,
            result.inputs.conf,
            result.params.isaTemp,
            result.params.tRef,
            result.params.tMax,
            result.params.tFlexMax,
            result.params.adjustedTora,
            result.params.headwind,
          )
        );
        break;
      case LimitingFactor.Vmcg:
        return (
          limitingWeights.altLimit -
          this.calculateVmcgTempDelta(
            temperature,
            result.inputs.conf,
            result.params.tRef,
            result.params.tMax,
            result.params.tFlexMax,
            result.params.adjustedTora,
            result.params.pressureAlt,
            result.params.isaTemp,
          ) -
          this.calculateVmcgWindDelta(
            temperature,
            result.inputs.conf,
            result.params.isaTemp,
            result.params.tRef,
            result.params.tMax,
            result.params.tFlexMax,
            result.params.adjustedTora,
            result.params.headwind,
          )
        );
        break;
    }
    return 0;
  }

  private calculateFlexTemp(
    result: Partial<TakeoffPerformanceResult>,
    tvmcg: number,
  ): [number | undefined, LimitingFactor | undefined] {
    if (
      !result.inputs ||
      !result.params ||
      !result.limits ||
      !result.tRefLimitingFactor ||
      !result.tMaxLimitingFactor ||
      !result.tFlexMaxLimitingFactor
    ) {
      throw new Error('Invalid result object!');
    }

    // we can use flex if TOW is below the tRef limit weight
    if (result.inputs.tow < result.limits[result.tRefLimitingFactor].tRefLimit) {
      let flexTemp: number | undefined;
      let flexLimitingFactor: LimitingFactor | undefined;
      let iterFrom: number;
      let iterTo: number;
      let fromLimitingFactor: LimitingFactor;
      let fromLimitingWeights: LimitWeight;
      let toLimitingFactor: LimitingFactor;
      let toLimitingWeights: LimitWeight;

      if (result.inputs.tow > result.limits[result.tMaxLimitingFactor].tMaxLimitNoBleed) {
        // interpolate between tRefLimit and tMaxLimit
        // that temp is highest flex temp
        iterFrom = result.params.tRef;
        iterTo = result.params.tMax;
        fromLimitingFactor = result.tRefLimitingFactor;
        fromLimitingWeights = result.limits[result.tRefLimitingFactor];
        toLimitingFactor = result.tMaxLimitingFactor;
        toLimitingWeights = result.limits[result.tMaxLimitingFactor];
      } else if (result.inputs.tow > result.limits[result.tFlexMaxLimitingFactor].tFlexMaxLimitNoBleed) {
        // interpolate between tMaxLimit and tFlexMaxLimit
        // that temp is highest flex temp
        iterFrom = result.params.tMax;
        iterTo = result.params.tFlexMax;
        fromLimitingFactor = result.tMaxLimitingFactor;
        fromLimitingWeights = result.limits[result.tMaxLimitingFactor];
        toLimitingFactor = result.tFlexMaxLimitingFactor;
        toLimitingWeights = result.limits[result.tFlexMaxLimitingFactor];
      } else {
        // interpolate between tFlexMax and tFlexMax + the maximum bleed increment,
        // so we can account for bleed corrections later, and only then limit the flex temp to tFlexMax
        iterFrom = result.params.tFlexMax;
        iterTo = result.params.tFlexMax + 8;
        fromLimitingFactor = result.tFlexMaxLimitingFactor;
        fromLimitingWeights = result.limits[result.tFlexMaxLimitingFactor];
        toLimitingFactor = result.tFlexMaxLimitingFactor;
        toLimitingWeights = fromLimitingWeights;
      }

      for (let t = iterFrom; t <= iterTo; t++) {
        const fromLimitTow = this.calculateFlexTow(result, fromLimitingFactor, fromLimitingWeights, t);
        const toLimitTow = this.calculateFlexTow(result, toLimitingFactor, toLimitingWeights, t);
        if (result.inputs.tow <= Math.min(fromLimitTow, toLimitTow)) {
          flexTemp = t;
          flexLimitingFactor = fromLimitTow <= toLimitTow ? fromLimitingFactor : toLimitingFactor;
        }
      }

      if (flexTemp !== undefined && flexLimitingFactor !== undefined) {
        if (result.inputs.antiIce === TakeoffAntiIceSetting.Engine) {
          flexTemp -= 2;
        } else if (result.inputs.antiIce === TakeoffAntiIceSetting.EngineWing) {
          flexTemp -= 6;
        }
        if (result.inputs.packs) {
          flexTemp -= 2;
        }

        flexTemp = Math.min(flexTemp, result.params.tFlexMax);

        flexTemp = Math.trunc(flexTemp);

        if (result.inputs.runwayCondition === RunwayCondition.Wet) {
          // tvmcg is already calculated when we calculate MTOW
          const factors: ReadonlyFloat64Array = (
            result.inputs.oat > tvmcg
              ? A380482xTakeoffPerformanceCalculator.wetFlexAdjustmentFactorsAboveTvmcg
              : A380482xTakeoffPerformanceCalculator.wetFlexAdjustmentFactorsAtOrBelowTvmcg
          )[result.inputs.conf].get(A380482xTakeoffPerformanceCalculator.vec4Cache, result.params.headwind);

          const lengthAltCoef = result.params.adjustedTora - result.params.pressureAlt / 20;
          const wetFlexAdjustment = Math.min(
            0,
            factors[0] * lengthAltCoef + factors[1],
            factors[2] * lengthAltCoef + factors[3],
          );
          flexTemp -= wetFlexAdjustment;
        }

        if (flexTemp > result.inputs.oat) {
          return [flexTemp, flexLimitingFactor];
        }
      }
    }

    return [undefined, undefined];
  }

  private calculateVSpeeds(
    result: Partial<TakeoffPerformanceResult>,
    applyForwardCgSpeedCorrection: boolean,
    tvmcg: number,
  ): void {
    if (!result.inputs || !result.params) {
      throw new Error('Invalid result object!');
    }

    if (result.inputs.runwayCondition == RunwayCondition.Dry || result.inputs.runwayCondition === RunwayCondition.Wet) {
      this.calculateDryVSpeeds(result, applyForwardCgSpeedCorrection);
      if (result.inputs.runwayCondition === RunwayCondition.Dry) {
        result.v1 = result.intermediateSpeeds?.dryV1;
        result.vR = result.intermediateSpeeds?.dryVR;
        result.v2 = result.intermediateSpeeds?.dryV2;
      } else {
        // Wet
        if (!result.intermediateSpeeds) {
          throw new Error('No dry speeds!');
        }
        // tvmcg is already calculated when we calculate MTOW
        const v1Factors: ReadonlyFloat64Array = (
          result.inputs.oat > tvmcg
            ? A380482xTakeoffPerformanceCalculator.wetV1AdjustmentFactorsAboveTvmcg
            : A380482xTakeoffPerformanceCalculator.wetV1AdjustmentFactorsAtOrBelowTvmcg
        )[result.inputs.conf].get(A380482xTakeoffPerformanceCalculator.vec4Cache, result.params.headwind);
        const vRFactors: ReadonlyFloat64Array | undefined = (
          result.inputs.oat > tvmcg
            ? undefined
            : A380482xTakeoffPerformanceCalculator.wetVRAdjustmentFactorsAtOrBelowTvmcg
        )?.[result.inputs.conf].get(A380482xTakeoffPerformanceCalculator.vec4Cache, result.params.headwind);
        const v2Factors: ReadonlyFloat64Array | undefined = (
          result.inputs.oat > tvmcg
            ? undefined
            : A380482xTakeoffPerformanceCalculator.wetV2AdjustmentFactorsAtOrBelowTvmcg
        )?.[result.inputs.conf].get(A380482xTakeoffPerformanceCalculator.vec4Cache, result.params.headwind);

        const lengthAltCoef = result.params.adjustedTora - result.params.pressureAlt / 20;
        const wetV1Adjustment = Math.min(
          0,
          v1Factors[0] * lengthAltCoef + v1Factors[1],
          v1Factors[2] * lengthAltCoef + v1Factors[3],
        );
        const wetVRAdjustment = vRFactors
          ? Math.min(0, vRFactors[0] * lengthAltCoef + vRFactors[1], vRFactors[2] * lengthAltCoef + vRFactors[3])
          : 0;
        const wetV2Adjustment = v2Factors
          ? Math.min(0, v2Factors[0] * lengthAltCoef + v2Factors[1], v2Factors[2] * lengthAltCoef + v2Factors[3])
          : 0;

        [result.v1, result.vR, result.v2] = this.reconcileVSpeeds(
          result,
          result.intermediateSpeeds.dryV1 - wetV1Adjustment,
          result.intermediateSpeeds.dryVR - wetVRAdjustment,
          result.intermediateSpeeds.dryV2 - wetV2Adjustment,
        );
      }
      return;
    }

    // otherwise the runway is contaminated

    let contamVSpeeds: Record<number, LerpVectorLookupTable>;
    switch (result.inputs.runwayCondition) {
      case RunwayCondition.Contaminated6mmWater:
        contamVSpeeds = A380482xTakeoffPerformanceCalculator.vSpeedsContaminated6mmWater;
        break;
      case RunwayCondition.Contaminated13mmWater:
        contamVSpeeds = A380482xTakeoffPerformanceCalculator.vSpeedsContaminated13mmWater;
        break;
      case RunwayCondition.Contaminated6mmSlush:
        contamVSpeeds = A380482xTakeoffPerformanceCalculator.vSpeedsContaminated6mmSlush;
        break;
      case RunwayCondition.Contaminated13mmSlush:
        contamVSpeeds = A380482xTakeoffPerformanceCalculator.vSpeedsContaminated13mmSlush;
        break;
      case RunwayCondition.ContaminatedCompactedSnow:
        contamVSpeeds = A380482xTakeoffPerformanceCalculator.vSpeedsContaminatedCompactedSnow;
        break;
      case RunwayCondition.Contaminated5mmWetSnow:
        contamVSpeeds = A380482xTakeoffPerformanceCalculator.vSpeedsContaminated5mmWetSnow;
        break;
      case RunwayCondition.Contaminated15mmWetSnow:
        contamVSpeeds = A380482xTakeoffPerformanceCalculator.vSpeedsContaminated15mmWetSnow;
        break;
      case RunwayCondition.Contaminated10mmDrySnow:
        contamVSpeeds = A380482xTakeoffPerformanceCalculator.vSpeedsContaminated10mmDrySnow;
        break;
      case RunwayCondition.Contaminated100mmDrySnow:
        contamVSpeeds = A380482xTakeoffPerformanceCalculator.vSpeedsContaminated100mmDrySnow;
        break;
      default:
        throw new Error('Invalid runway condition');
    }

    const [v1, vR, v2] = contamVSpeeds[result.inputs.conf].get(
      A380482xTakeoffPerformanceCalculator.vec3Cache,
      result.inputs.tow,
    );

    [result.v1, result.vR, result.v2] = this.reconcileVSpeeds(result, v1, vR, v2);
  }

  private calculateDryVSpeeds(result: Partial<TakeoffPerformanceResult>, applyForwardCgSpeedCorrection: boolean): void {
    if (!result.inputs || !result.params) {
      throw new Error('Invalid result object!');
    }

    let v1: number;
    let vR: number;
    let v2: number;

    const speeds: Partial<TakeoffPerformanceSpeeds> = {};

    const limitingFactor =
      result.params.flexLimitingFactor !== undefined ? result.params.flexLimitingFactor : result.oatLimitingFactor;

    if (limitingFactor === LimitingFactor.Runway || limitingFactor === LimitingFactor.Vmcg) {
      // v2
      const [v2BaseFactor1, v2BaseFactor2]: [number, number] =
        A380482xTakeoffPerformanceCalculator.v2RunwayVmcgBaseFactors[result.inputs.conf];
      speeds.v2Base = (result.inputs.tow / 1000) * v2BaseFactor1 + v2BaseFactor2;

      const [v2AltFactor1, v2AltFactor2] =
        A380482xTakeoffPerformanceCalculator.v2RunwayVmcgAltFactors[result.inputs.conf];
      speeds.v2DeltaAlt = ((result.inputs.tow / 1000) * v2AltFactor1 + v2AltFactor2) * result.params.pressureAlt;

      v2 = speeds.v2Base + speeds.v2DeltaAlt;

      // vr
      const [vRBaseFactor1, vRBaseFactor2]: [number, number] =
        A380482xTakeoffPerformanceCalculator.vRRunwayVmcgBaseFactors[result.inputs.conf];
      speeds.vRBase = (result.inputs.tow / 1000) * vRBaseFactor1 + vRBaseFactor2;

      const [vRBaseLength, vRRunwayFactor1, vRRunwayFactor2] =
        A380482xTakeoffPerformanceCalculator.vRRunwayVmcgRunwayFactors[result.inputs.conf];
      speeds.vRDeltaRunway =
        (vRBaseLength - result.params.adjustedTora) * ((result.inputs.tow / 1000) * vRRunwayFactor1 + vRRunwayFactor2);

      const [vRFactorAlt1, vRFactorAlt2] =
        A380482xTakeoffPerformanceCalculator.vRRunwayVmcgAltFactors[result.inputs.conf];
      speeds.vRDeltaAlt = result.params.pressureAlt * ((result.inputs.tow / 1000) * vRFactorAlt1 + vRFactorAlt2);

      const vRFactorSlope = A380482xTakeoffPerformanceCalculator.vRRunwayVmcgSlopeFactors[result.inputs.conf];
      speeds.vRDeltaSlope = result.inputs.slope * result.params.adjustedTora * vRFactorSlope;

      const [vRWindFactor1, vRWindFactor2] =
        result.params.headwind >= 0
          ? A380482xTakeoffPerformanceCalculator.vRRunwayVmcgHeadwindFactors[result.inputs.conf]
          : A380482xTakeoffPerformanceCalculator.vRRunwayVmcgTailwindFactors[result.inputs.conf];
      speeds.vRDeltaWind = result.params.headwind * ((result.inputs.tow / 1000) * vRWindFactor1 + vRWindFactor2);

      vR =
        speeds.vRBase +
        speeds.vRDeltaRunway +
        speeds.vRDeltaAlt +
        speeds.vRDeltaSlope +
        speeds.vRDeltaWind +
        (applyForwardCgSpeedCorrection ? -1 : 0);

      // v1
      const [v1BaseFactor1, v1BaseFactor2]: [number, number] =
        A380482xTakeoffPerformanceCalculator.v1RunwayVmcgBaseFactors[result.inputs.conf];
      speeds.v1Base = (result.inputs.tow / 1000) * v1BaseFactor1 + v1BaseFactor2;

      const [v1BaseLength, v1RunwayFactor1, v1RunwayFactor2] =
        A380482xTakeoffPerformanceCalculator.v1RunwayVmcgRunwayFactors[result.inputs.conf];
      speeds.v1DeltaRunway =
        (v1BaseLength - result.params.adjustedTora) * ((result.inputs.tow / 1000) * v1RunwayFactor1 + v1RunwayFactor2);

      const [v1FactorAlt1, v1FactorAlt2] =
        A380482xTakeoffPerformanceCalculator.v1RunwayVmcgAltFactors[result.inputs.conf];
      speeds.v1DeltaAlt = result.params.pressureAlt * ((result.inputs.tow / 1000) * v1FactorAlt1 + v1FactorAlt2);

      const v1FactorSlope = A380482xTakeoffPerformanceCalculator.v1RunwayVmcgSlopeFactors[result.inputs.conf];
      speeds.v1DeltaSlope = result.inputs.slope * result.params.adjustedTora * v1FactorSlope;

      const [v1WindFactor1, v1WindFactor2] =
        result.params.headwind >= 0
          ? A380482xTakeoffPerformanceCalculator.v1RunwayVmcgHeadwindFactors[result.inputs.conf]
          : A380482xTakeoffPerformanceCalculator.v1RunwayVmcgTailwindFactors[result.inputs.conf];
      speeds.v1DeltaWind = result.params.headwind * ((result.inputs.tow / 1000) * v1WindFactor1 + v1WindFactor2);

      v1 = speeds.v1Base + speeds.v1DeltaRunway + speeds.v1DeltaAlt + speeds.v1DeltaSlope + speeds.v1DeltaWind;
    } else {
      // 2nd seg or brake energy limited
      // v2
      const v2NoWind = this.calculateSecondSegBrakeV2(speeds, result, false, false);
      const [v2ThresholdFactor1, v2ThresholdFactor2] =
        A380482xTakeoffPerformanceCalculator.v2SecondSegBrakeThresholds[result.inputs.conf];
      speeds.v2Table2Threshold = result.params.adjustedTora * v2ThresholdFactor1 + v2ThresholdFactor2;
      const useTable2 = v2NoWind >= speeds.v2Table2Threshold;
      if (useTable2) {
        speeds.v2Table1NoWind = v2NoWind;
      }
      v2 = this.calculateSecondSegBrakeV2(speeds, result, true, useTable2);

      // vr
      const [vRBaseFactor1, vRBaseFactor2]: [number, number] = useTable2
        ? A380482xTakeoffPerformanceCalculator.vRSecondSegBrakeBaseTable2[result.inputs.conf]
        : A380482xTakeoffPerformanceCalculator.vRSecondSegBrakeBaseTable1[result.inputs.conf];
      speeds.vRBase = (result.inputs.tow / 1000) * vRBaseFactor1 + vRBaseFactor2;

      const [vRBaseLength, vRRunwayFactor1, vRRunwayFactor2] = useTable2
        ? A380482xTakeoffPerformanceCalculator.vRSecondSegBrakeRunwayTable2[result.inputs.conf]
        : A380482xTakeoffPerformanceCalculator.vRSecondSegBrakeRunwayTable1[result.inputs.conf];
      speeds.vRDeltaRunway =
        (vRBaseLength - result.params.adjustedTora) * ((result.inputs.tow / 1000) * vRRunwayFactor1 + vRRunwayFactor2);

      const [vRAltFactor1, vRAltFactor2, vRAltFactor3, vRAltFactor4] = useTable2
        ? A380482xTakeoffPerformanceCalculator.vRSecondSegBrakeAltTable2[result.inputs.conf]
        : A380482xTakeoffPerformanceCalculator.vRSecondSegBrakeAltTable1[result.inputs.conf];
      speeds.vRDeltaAlt =
        result.params.pressureAlt *
        ((result.inputs.tow / 1000) * vRAltFactor1 + vRAltFactor2) *
        (result.params.adjustedTora * vRAltFactor3 + vRAltFactor4);

      const [vRSlopeFactor1, vRSlopeFactor2] =
        A380482xTakeoffPerformanceCalculator.vRSecondSegBrakeSlopeFactors[result.inputs.conf];
      speeds.vRDeltaSlope =
        result.inputs.slope *
        result.params.adjustedTora *
        ((result.inputs.tow / 1000) * vRSlopeFactor1 + vRSlopeFactor2);

      const [vRWindFactor1, vRWindFactor2] =
        result.params.headwind >= 0
          ? A380482xTakeoffPerformanceCalculator.vRSecondSegBrakeHeadwindFactors[result.inputs.conf]
          : A380482xTakeoffPerformanceCalculator.vRSecondSegBrakeTailwindFactors[result.inputs.conf];
      speeds.vRDeltaWind = result.params.headwind * ((result.inputs.tow / 1000) * vRWindFactor1 + vRWindFactor2);

      vR = speeds.vRBase + speeds.vRDeltaRunway + speeds.vRDeltaAlt + speeds.vRDeltaSlope + speeds.vRDeltaWind;

      // v1
      // FIXME temp workaround noted in the document
      v1 = this.calculateSecondSegBrakeV1(speeds, result, useTable2);
      if (useTable2 && v2 - v1 > 8) {
        speeds.v1Table2 = v1;
        v1 = this.calculateSecondSegBrakeV1(speeds, result, false);
      }
    }

    [speeds.dryV1, speeds.dryVR, speeds.dryV2] = this.reconcileVSpeeds(result, v1, vR, v2);

    result.intermediateSpeeds = speeds as TakeoffPerformanceSpeeds;
  }

  private calculateSecondSegBrakeV2(
    speeds: Partial<TakeoffPerformanceSpeeds>,
    result: Partial<TakeoffPerformanceResult>,
    correctWind: boolean,
    useTable2: boolean,
  ): number {
    if (!result.inputs || !result.params) {
      throw new Error('Invalid result object!');
    }

    const [v2BaseFactor1, v2BaseFactor2] =
      A380482xTakeoffPerformanceCalculator.v2SecondSegBrakeBaseTable1[result.inputs.conf];
    speeds.v2Base = (result.inputs.tow / 1000) * v2BaseFactor1 + v2BaseFactor2;

    if (useTable2) {
      const [v2BaseFactor1, v2BaseFactor2] =
        A380482xTakeoffPerformanceCalculator.v2SecondSegBrakeBaseTable2[result.inputs.conf];
      speeds.v2Base = (result.inputs.tow / 1000) * v2BaseFactor1 + v2BaseFactor2;
    }

    const [v2BaseRunwayLength, v2RunwayFactor] = useTable2
      ? A380482xTakeoffPerformanceCalculator.v2SecondSegBrakeRunwayTable2[result.inputs.conf]
      : A380482xTakeoffPerformanceCalculator.v2SecondSegBrakeRunwayTable1[result.inputs.conf];
    speeds.v2DeltaRunway = (v2BaseRunwayLength - result.params.adjustedTora) * v2RunwayFactor;

    const [v2AltFactor1, v2AltFactor2, v2AltFactor3, v2AltFactor4] =
      A380482xTakeoffPerformanceCalculator.v2SecondSegBrakeAltFactors[result.inputs.conf];
    speeds.v2DeltaAlt =
      result.params.pressureAlt *
      ((result.inputs.tow / 1000) * v2AltFactor1 + v2AltFactor2) *
      (result.params.adjustedTora * v2AltFactor3 + v2AltFactor4);

    const [v2SlopeFactor1, v2SlopeFactor2] =
      A380482xTakeoffPerformanceCalculator.v2SecondSegBrakeSlopeFactors[result.inputs.conf];
    speeds.v2DeltaSlope =
      result.inputs.slope * result.params.adjustedTora * ((result.inputs.tow / 1000) * v2SlopeFactor1 + v2SlopeFactor2);

    if (correctWind) {
      const v2WindFactor =
        result.params.headwind >= 0
          ? A380482xTakeoffPerformanceCalculator.v2SecondSegBrakeHeadwindFactors[result.inputs.conf]
          : A380482xTakeoffPerformanceCalculator.v2SecondSegBrakeTailwindFactors[result.inputs.conf];
      speeds.v2DeltaWind = result.params.headwind * v2WindFactor;
    } else {
      speeds.v2DeltaWind = 0;
    }

    return speeds.v2Base + speeds.v2DeltaRunway + speeds.v2DeltaAlt + speeds.v2DeltaSlope + speeds.v2DeltaWind;
  }

  private calculateSecondSegBrakeV1(
    speeds: Partial<TakeoffPerformanceSpeeds>,
    result: Partial<TakeoffPerformanceResult>,
    useTable2: boolean,
  ): number {
    if (!result.inputs || !result.params) {
      throw new Error('Invalid result object!');
    }

    const [v1BaseFactor1, v1BaseFactor2]: [number, number] = useTable2
      ? A380482xTakeoffPerformanceCalculator.v1SecondSegBrakeBaseTable2[result.inputs.conf]
      : A380482xTakeoffPerformanceCalculator.v1SecondSegBrakeBaseTable1[result.inputs.conf];
    speeds.v1Base = (result.inputs.tow / 1000) * v1BaseFactor1 + v1BaseFactor2;

    const [v1BaseLength, v1RunwayFactor1, v1RunwayFactor2] = useTable2
      ? A380482xTakeoffPerformanceCalculator.v1SecondSegBrakeRunwayTable2[result.inputs.conf]
      : A380482xTakeoffPerformanceCalculator.v1SecondSegBrakeRunwayTable1[result.inputs.conf];
    speeds.v1DeltaRunway =
      (v1BaseLength - result.params.adjustedTora) * ((result.inputs.tow / 1000) * v1RunwayFactor1 + v1RunwayFactor2);

    const [v1AltFactor1, v1AltFactor2, v1AltFactor3, v1AltFactor4] = useTable2
      ? A380482xTakeoffPerformanceCalculator.v1SecondSegBrakeAltTable2[result.inputs.conf]
      : A380482xTakeoffPerformanceCalculator.v1SecondSegBrakeAltTable1[result.inputs.conf];
    speeds.v1DeltaAlt =
      result.params.pressureAlt *
      ((result.inputs.tow / 1000) * v1AltFactor1 + v1AltFactor2) *
      (result.params.adjustedTora * v1AltFactor3 + v1AltFactor4);

    const [v1SlopeFactor1, v1SlopeFactor2] =
      A380482xTakeoffPerformanceCalculator.v1SecondSegBrakeSlopeFactors[result.inputs.conf];
    speeds.v1DeltaSlope =
      result.inputs.slope * result.params.adjustedTora * ((result.inputs.tow / 1000) * v1SlopeFactor1 + v1SlopeFactor2);

    const [v1WindFactor1, v1WindFactor2] =
      result.params.headwind >= 0
        ? A380482xTakeoffPerformanceCalculator.v1SecondSegBrakeHeadwindFactors[result.inputs.conf]
        : A380482xTakeoffPerformanceCalculator.v1SecondSegBrakeTailwindFactors[result.inputs.conf];
    speeds.v1DeltaWind = result.params.headwind * ((result.inputs.tow / 1000) * v1WindFactor1 + v1WindFactor2);

    return speeds.v1Base + speeds.v1DeltaRunway + speeds.v1DeltaAlt + speeds.v1DeltaSlope + speeds.v1DeltaWind;
  }

  private reconcileVSpeeds(
    result: Partial<TakeoffPerformanceResult>,
    v1: number,
    vR: number,
    v2: number,
  ): [number, number, number] {
    if (!result.inputs || !result.params) {
      throw new Error('Invalid result object!');
    }

    const minV1Vmc = Math.ceil(A380482xTakeoffPerformanceCalculator.minimumV1Vmc.get(result.params.pressureAlt));
    const minVrVmc = Math.ceil(A380482xTakeoffPerformanceCalculator.minimumVrVmc.get(result.params.pressureAlt));
    const minV2Vmc = Math.ceil(
      A380482xTakeoffPerformanceCalculator.minimumV2Vmc[result.inputs.conf].get(result.params.pressureAlt),
    );
    const minv2Vmu = Math.ceil(
      A380482xTakeoffPerformanceCalculator.minimumV2Vmu[result.inputs.conf].get(
        result.params.pressureAlt,
        result.inputs.tow,
      ),
    );

    // VMCG/VMCA/VMU limits, and rounding
    let v1Corrected = Math.round(Math.max(v1, minV1Vmc));
    let vRCorrected = Math.round(Math.max(vR, minVrVmc));
    const v2Corrected = Math.round(Math.max(v2, minV2Vmc, minv2Vmu));

    // Vr must be less than or equal to v2
    if (vRCorrected > v2Corrected) {
      vRCorrected = v2Corrected;
      if (vRCorrected < minVrVmc) {
        result.error = TakeoffPerfomanceError.VmcgVmcaLimits;
      }
    }

    // Vr limited by tire speed
    if (v2Corrected > 195) {
      const maxVr = Math.trunc(195 - (v2Corrected - 195));
      if (vRCorrected > 195) {
        result.error = TakeoffPerfomanceError.MaximumTireSpeed;
      } else if (vRCorrected > maxVr) {
        vRCorrected = maxVr;
        if (vRCorrected < minVrVmc) {
          result.error = TakeoffPerfomanceError.VmcgVmcaLimits;
        }
      }
    }

    // V1 must be less than or equal to vr
    if (v1Corrected > vRCorrected) {
      v1Corrected = vRCorrected;
      if (v1Corrected < minV1Vmc) {
        result.error = TakeoffPerfomanceError.VmcgVmcaLimits;
      }
    }

    return [v1Corrected, vRCorrected, v2Corrected];
  }

  private calculateWeightLimits(
    limitingFactor: LimitingFactor,
    result: Partial<TakeoffPerformanceResult>,
  ): LimitWeight {
    if (!result.inputs || !result.params) {
      throw new Error('Invalid result object!');
    }

    const weights: Partial<LimitWeight> = {};

    let baseFactors: typeof A380482xTakeoffPerformanceCalculator.secondSegmentBaseFactor | undefined;
    let slopeFactors: typeof A380482xTakeoffPerformanceCalculator.runwaySlopeFactor;
    let altFactors: typeof A380482xTakeoffPerformanceCalculator.runwayPressureAltFactor;
    let tempDeltaFunc: typeof this.calculateRunwayTempDelta;
    let windDeltaFunc: typeof this.calculateRunwayWindDelta;

    switch (limitingFactor) {
      case LimitingFactor.Runway:
        // no base factors
        slopeFactors = A380482xTakeoffPerformanceCalculator.runwaySlopeFactor;
        altFactors = A380482xTakeoffPerformanceCalculator.runwayPressureAltFactor;
        tempDeltaFunc = this.calculateRunwayTempDelta;
        windDeltaFunc = this.calculateRunwayWindDelta;
        break;
      case LimitingFactor.SecondSegment:
        baseFactors = A380482xTakeoffPerformanceCalculator.secondSegmentBaseFactor;
        slopeFactors = A380482xTakeoffPerformanceCalculator.secondSegmentSlopeFactor;
        altFactors = A380482xTakeoffPerformanceCalculator.secondSegmentPressureAltFactor;
        tempDeltaFunc = this.calculateSecondSegmentTempDelta;
        windDeltaFunc = this.calculateSecondSegmentWindDelta;
        break;
      case LimitingFactor.BrakeEnergy:
        baseFactors = A380482xTakeoffPerformanceCalculator.brakeEnergyBaseFactor;
        slopeFactors = A380482xTakeoffPerformanceCalculator.brakeEnergySlopeFactor;
        altFactors = A380482xTakeoffPerformanceCalculator.brakeEnergyPressureAltFactor;
        tempDeltaFunc = this.calculateBrakeEnergyTempDelta;
        windDeltaFunc = this.calculateBrakeEnergyWindDelta;
        break;
      case LimitingFactor.Vmcg:
        baseFactors = A380482xTakeoffPerformanceCalculator.vmcgBaseFactor;
        slopeFactors = A380482xTakeoffPerformanceCalculator.vmcgSlopeFactor;
        altFactors = A380482xTakeoffPerformanceCalculator.vmcgPressureAltFactor;
        tempDeltaFunc = this.calculateVmcgTempDelta;
        windDeltaFunc = this.calculateVmcgWindDelta;
        break;
      default:
        throw new Error('Invalid limiting factor!');
    }

    // base weight limits at sea level, isa etc.
    if (limitingFactor === LimitingFactor.Runway) {
      weights.baseLimit = this.calculateBaseRunwayPerfLimit(result.params.adjustedTora, result.inputs.conf);
    } else {
      if (!baseFactors) {
        throw new Error('Missing base factors!');
      }
      weights.baseLimit = this.calculateBaseLimit(result.params.adjustedTora, result.inputs.conf, baseFactors);
    }

    // correction for runway slope dependent on config
    // note: downhill = increased weight limit
    weights.deltaSlope = 1000 * slopeFactors[result.inputs.conf] * result.params.adjustedTora * result.inputs.slope;
    weights.slopeLimit = weights.baseLimit - weights.deltaSlope;

    // correction for pressure altitude
    const [altFactor1, altFactor2] = altFactors[result.inputs.conf];
    weights.deltaAlt = 1000 * result.params.pressureAlt * (result.params.pressureAlt * altFactor1 + altFactor2);
    weights.altLimit = weights.slopeLimit - weights.deltaAlt;

    // correction for bleeds
    const deltaBleed =
      (result.inputs.antiIce === TakeoffAntiIceSetting.EngineWing ? 1_600 : 0) + (result.inputs.packs ? 1_500 : 0);

    // correction for air temperature and wind
    weights.oatDeltaTemp = tempDeltaFunc(
      result.inputs.oat,
      result.inputs.conf,
      result.params.tRef,
      result.params.tMax,
      result.params.tFlexMax,
      result.params.adjustedTora,
      result.params.pressureAlt,
      result.params.isaTemp,
    );
    weights.oatDeltaWind = windDeltaFunc(
      result.inputs.oat,
      result.inputs.conf,
      result.params.isaTemp,
      result.params.tRef,
      result.params.tMax,
      result.params.tFlexMax,
      result.params.adjustedTora,
      result.params.headwind,
    );
    weights.oatLimitNoBleed = weights.altLimit - weights.oatDeltaTemp - weights.oatDeltaWind;
    weights.oatLimit = weights.oatLimitNoBleed - deltaBleed;

    weights.tRefDeltaTemp = tempDeltaFunc(
      result.params.tRef,
      result.inputs.conf,
      result.params.tRef,
      result.params.tMax,
      result.params.tFlexMax,
      result.params.adjustedTora,
      result.params.pressureAlt,
      result.params.isaTemp,
    );
    weights.tRefDeltaWind = windDeltaFunc(
      result.params.tRef,
      result.inputs.conf,
      result.params.isaTemp,
      result.params.tRef,
      result.params.tMax,
      result.params.tFlexMax,
      result.params.adjustedTora,
      result.params.headwind,
    );
    weights.tRefLimitNoBleed = weights.altLimit - weights.tRefDeltaTemp - weights.tRefDeltaWind;
    weights.tRefLimit = weights.tRefLimitNoBleed - deltaBleed;

    weights.tMaxDeltaTemp = tempDeltaFunc(
      result.params.tMax,
      result.inputs.conf,
      result.params.tRef,
      result.params.tMax,
      result.params.tFlexMax,
      result.params.adjustedTora,
      result.params.pressureAlt,
      result.params.isaTemp,
    );
    weights.tMaxDeltaWind = windDeltaFunc(
      result.params.tMax,
      result.inputs.conf,
      result.params.isaTemp,
      result.params.tRef,
      result.params.tMax,
      result.params.tFlexMax,
      result.params.adjustedTora,
      result.params.headwind,
    );
    weights.tMaxLimitNoBleed = weights.altLimit - weights.tMaxDeltaTemp - weights.tMaxDeltaWind;
    weights.tMaxLimit = weights.tMaxLimitNoBleed - deltaBleed;

    weights.tFlexMaxDeltaTemp = tempDeltaFunc(
      result.params.tFlexMax,
      result.inputs.conf,
      result.params.tRef,
      result.params.tMax,
      result.params.tFlexMax,
      result.params.adjustedTora,
      result.params.pressureAlt,
      result.params.isaTemp,
    );
    weights.tFlexMaxDeltaWind = windDeltaFunc(
      result.params.tFlexMax,
      result.inputs.conf,
      result.params.isaTemp,
      result.params.tRef,
      result.params.tMax,
      result.params.tFlexMax,
      result.params.adjustedTora,
      result.params.headwind,
    );
    weights.tFlexMaxLimitNoBleed = weights.altLimit - weights.tFlexMaxDeltaTemp - weights.tFlexMaxDeltaWind;
    weights.tFlexMaxLimit = weights.tFlexMaxLimitNoBleed - deltaBleed;

    return weights as LimitWeight;
  }

  /**
   * Determine which of the factors is limiting the takeoff weight most for a given temperature.
   * @param temp The temperature to check.
   * @param result The partially calculated result.
   * @returns The most limiting factor.
   */
  private getLimitingFactor(
    temp: 'oatLimit' | 'tRefLimit' | 'tMaxLimit' | 'tFlexMaxLimit',
    result: Partial<TakeoffPerformanceResult>,
  ): LimitingFactor {
    if (!result.limits) {
      throw new Error('Invalid result object!');
    }

    let limitingWeight = Infinity;
    let limitingFactor = LimitingFactor.Runway;

    for (const factor of Object.values(LimitingFactor) as LimitingFactor[]) {
      const weights = result.limits[factor];
      if (weights !== undefined && weights[temp] < limitingWeight) {
        limitingWeight = weights[temp];
        limitingFactor = factor;
      }
    }

    return limitingFactor;
  }

  /** @inheritdoc */
  public calculateTakeoffPerformanceOptConf(
    tow: number,
    forwardCg: boolean,
    tora: number,
    slope: number,
    lineupAngle: LineupAngle,
    wind: number,
    elevation: number,
    qnh: number,
    oat: number,
    antiIce: TakeoffAntiIceSetting,
    packs: boolean,
    forceToga: boolean,
    runwayCondition: RunwayCondition,
    cg?: number,
    out?: Partial<TakeoffPerformanceResult>,
  ): TakeoffPerformanceResult {
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    const results = [1, 2, 3].map((conf, cacheIndex) =>
      this.calculateTakeoffPerformance(
        tow,
        forwardCg,
        conf,
        tora,
        slope,
        lineupAngle,
        wind,
        elevation,
        qnh,
        oat,
        antiIce,
        packs,
        forceToga,
        runwayCondition,
        cg,
        // A380482xTakeoffPerformanceCalculator.optResultCache[cacheIndex],
      ),
    );

    const filteredResults = results.filter((r) => r.error === TakeoffPerfomanceError.None);

    // if all the results failed, return the highest conf
    if (filteredResults.length === 0) {
      return A380482xTakeoffPerformanceCalculator.deepCopy(results[results.length - 1], out);
    }

    // pick the result with the highest flex temp, and the lowest speeds (if two have the same flex)
    filteredResults.sort((a, b) => (a.flex === b.flex ? (a.v1 ?? 0) - (b.v1 ?? 0) : (b.flex ?? 0) - (a.flex ?? 0)));
    return A380482xTakeoffPerformanceCalculator.deepCopy(filteredResults[0], out ?? {});
  }

  private static deepCopy(
    result: TakeoffPerformanceResult,
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    out?: Partial<TakeoffPerformanceResult>,
  ): TakeoffPerformanceResult {
    // FIXME implement properly
    return JSON.parse(JSON.stringify(result));
  }

  private calculateTvmcg(inputs: TakeoffPerformanceInputs, params: TakeoffPerformanceParameters): number {
    const factors: LerpVectorLookupTable = A380482xTakeoffPerformanceCalculator.tvmcgFactors[inputs.conf];
    const [factor1, factor2] = factors.get(
      A380482xTakeoffPerformanceCalculator.vec2Cache,
      Math.max(params.headwind, -15),
    );
    return factor1 * (params.adjustedTora - params.pressureAlt / 10) + factor2;
  }

  /**
   * Get the ISA temperature from elevation.
   * @param elevation Elevation in feet.
   * @returns ISA temperature in °C.
   */
  private calculateIsaTemp(elevation: number): number {
    return 15 - elevation * 0.0019812;
  }

  /**
   * Get the Tref temperature from elevation.
   * @param elevation Elevation in feet.
   * @returns Tref in °C.
   */
  private calculateTref(elevation: number): number {
    return A380482xTakeoffPerformanceCalculator.tRefTable.get(elevation);
  }

  /**
   * Get the Tmax temperature from elevation.
   * @param pressureAlt Pressure altitude in feet.
   * @returns Tmax in °C.
   */
  private calculateTmax(pressureAlt: number): number {
    return A380482xTakeoffPerformanceCalculator.tMaxTable.get(pressureAlt);
  }

  /**
   * Get the maximum flex temperature from ISA temp.
   * @param isa ISA temperature in °C.
   * @returns Tflexmax in °C.
   */
  private calculateTflexMax(isa: number): number {
    return isa + A380482xTakeoffPerformanceCalculator.tMaxFlexDisa;
  }

  private calculatePressureAltitude(elevation: number, qnh: number): number {
    return elevation + 145442.15 * (1 - (qnh / 1013.25) ** 0.190263);
  }

  private calculateBaseRunwayPerfLimit(length: number, conf: number): number {
    switch (conf) {
      case 1:
        return A380482xTakeoffPerformanceCalculator.runwayPerfLimitConf1.get(length);
      case 2:
        return A380482xTakeoffPerformanceCalculator.runwayPerfLimitConf2.get(length);
      case 3:
        return A380482xTakeoffPerformanceCalculator.runwayPerfLimitConf3.get(length);
      default:
        return NaN;
    }
  }

  private calculateBaseLimit(length: number, conf: number, factors: Record<number, [number, number]>): number {
    const [factor1, factor2] = factors[conf];
    return 1000 * (length * factor1 + factor2);
  }

  /** Calculates the temperature correction in kg. */
  private calculateRunwayTempDelta(
    temp: number,
    conf: number,
    tRef: number,
    tMax: number,
    tFlexMax: number,
    runwayLength: number,
    pressureAlt: number,
    isaTemp: number,
  ): number {
    if (temp > tFlexMax) {
      return NaN;
    }

    const tempFactors = A380482xTakeoffPerformanceCalculator.runwayTemperatureFactor[conf];

    const runwayAltFactor = runwayLength - pressureAlt / 12;
    let weightDelta = 1000 * (runwayAltFactor * tempFactors[0] + tempFactors[1]) * (Math.min(temp, tRef) - isaTemp);
    if (temp > tRef) {
      weightDelta += 1000 * (runwayAltFactor * tempFactors[2] + tempFactors[3]) * (Math.min(temp, tMax) - tRef);
    }
    if (temp > tMax) {
      weightDelta += 1000 * (runwayAltFactor * tempFactors[4] + tempFactors[5]) * (temp - tMax);
    }
    return weightDelta;
  }

  /** Calculates the temperature correction in kg. */
  private calculateSecondSegmentTempDelta(
    temp: number,
    conf: number,
    tRef: number,
    tMax: number,
    tFlexMax: number,
    runwayLength: number,
    pressureAlt: number,
    isaTemp: number,
  ): number {
    if (temp > tFlexMax) {
      return NaN;
    }

    const tempFactors = A380482xTakeoffPerformanceCalculator.secondSegmentTemperatureFactor[conf];

    let weightDelta =
      1000 * ((runwayLength - pressureAlt / 5) * tempFactors[0] + tempFactors[1]) * (Math.min(temp, tRef) - isaTemp);
    if (temp > tRef) {
      weightDelta +=
        1000 * ((runwayLength - pressureAlt / 5) * tempFactors[2] + tempFactors[3]) * (Math.min(temp, tMax) - tRef);
    }
    if (temp > tMax) {
      weightDelta += 1000 * ((runwayLength - pressureAlt / 5) * tempFactors[4] + tempFactors[5]) * (temp - tMax);
    }
    return weightDelta;
  }

  /** Calculates the temperature correction in kg. */
  private calculateBrakeEnergyTempDelta(
    temp: number,
    conf: number,
    tRef: number,
    tMax: number,
    tFlexMax: number,
    runwayLength: number,
    pressureAlt: number,
    isaTemp: number,
  ): number {
    if (temp > tFlexMax) {
      return NaN;
    }

    const tempFactors = A380482xTakeoffPerformanceCalculator.brakeEnergyTemperatureFactor[conf];

    let weightDelta = 1000 * tempFactors[0] * (Math.min(temp, tRef) - isaTemp);
    if (temp > tRef) {
      weightDelta += 1000 * tempFactors[1] * (Math.min(temp, tMax) - tRef);
    }
    // no correction above Tmax
    return weightDelta;
  }

  /** Calculates the temperature correction in kg. */
  private calculateVmcgTempDelta(
    temp: number,
    conf: number,
    tRef: number,
    tMax: number,
    tFlexMax: number,
    runwayLength: number,
    pressureAlt: number,
    isaTemp: number,
  ): number {
    if (temp > tFlexMax) {
      return NaN;
    }

    const tempFactors = A380482xTakeoffPerformanceCalculator.vmcgTemperatureFactor[conf];

    let weightDelta = 1000 * (runwayLength * tempFactors[0] + tempFactors[1]) * (Math.min(temp, tRef) - isaTemp);
    if (temp > tRef) {
      weightDelta += 1000 * (runwayLength * tempFactors[2] + tempFactors[3]) * (Math.min(temp, tMax) - tRef);
    }
    if (temp > tMax) {
      weightDelta += 1000 * (runwayLength * tempFactors[4] + tempFactors[5]) * (temp - tMax);
    }
    return weightDelta;
  }

  /** Calculates the wind correction in kg, -ve is a positive increment on the limit weight (deltas are subtracted). */
  private calculateRunwayWindDelta(
    temp: number,
    conf: number,
    isaTemp: number,
    tRef: number,
    tMax: number,
    tFlexMax: number,
    runwayLength: number,
    wind: number,
  ): number {
    if (temp > tFlexMax) {
      return NaN;
    }

    const windFactors =
      wind >= 0
        ? A380482xTakeoffPerformanceCalculator.runwayHeadWindFactor[conf]
        : A380482xTakeoffPerformanceCalculator.runwayTailWindFactor[conf];

    let weightDelta = 1000 * (runwayLength * windFactors[0] + windFactors[1]) * wind;
    if (temp > tRef) {
      weightDelta += 1000 * windFactors[2] * wind * (Math.min(temp, tMax) - tRef);
    }
    if (temp > tMax) {
      weightDelta += 1000 * windFactors[3] * wind * (temp - tMax);
    }

    // cover an edge case near the ends of the data
    if (Math.sign(weightDelta) === Math.sign(wind)) {
      return 0;
    }
    return weightDelta;
  }

  /** Calculates the wind correction in kg, -ve is a positive increment on the limit weight (deltas are subtracted). */
  private calculateSecondSegmentWindDelta(
    temp: number,
    conf: number,
    isaTemp: number,
    tRef: number,
    tMax: number,
    tFlexMax: number,
    runwayLength: number,
    wind: number,
  ): number {
    if (temp > tFlexMax) {
      return NaN;
    }

    const windFactors =
      wind >= 0
        ? A380482xTakeoffPerformanceCalculator.secondSegmentHeadWindFactor[conf]
        : A380482xTakeoffPerformanceCalculator.secondSegmentTailWindFactor[conf];

    let weightDelta = 1000 * (runwayLength * windFactors[0] + windFactors[1]) * wind;
    if (temp > tRef) {
      weightDelta += 1000 * windFactors[2] * wind * (Math.min(temp, tMax) - tRef);
    }
    if (temp > tMax) {
      weightDelta += 1000 * windFactors[3] * wind * (temp - tMax);
    }

    // cover an edge case near the ends of the data
    if (Math.sign(weightDelta) === Math.sign(wind)) {
      return 0;
    }
    return weightDelta;
  }

  /** Calculates the wind correction in kg, -ve is a positive increment on the limit weight (deltas are subtracted). */
  private calculateBrakeEnergyWindDelta(
    temp: number,
    conf: number,
    isaTemp: number,
    tRef: number,
    tMax: number,
    tFlexMax: number,
    runwayLength: number,
    wind: number,
  ): number {
    if (temp > tFlexMax) {
      return NaN;
    }

    const windFactors =
      wind >= 0
        ? A380482xTakeoffPerformanceCalculator.brakeEnergyHeadWindFactor[conf]
        : A380482xTakeoffPerformanceCalculator.brakeEnergyTailWindFactor[conf];

    let weightDelta = 1000 * (runwayLength * windFactors[0] + windFactors[1]) * wind;
    if (temp > tRef) {
      weightDelta += 1000 * windFactors[2] * wind * (Math.min(temp, tMax) - tRef);
    }
    if (temp > tMax) {
      weightDelta += 1000 * windFactors[3] * wind * (temp - tMax);
    }

    // cover an edge case near the ends of the data
    if (Math.sign(weightDelta) === Math.sign(wind)) {
      return 0;
    }
    return weightDelta;
  }

  /** Calculates the wind correction in kg, -ve is a positive increment on the limit weight (deltas are subtracted). */
  private calculateVmcgWindDelta(
    temp: number,
    conf: number,
    isaTemp: number,
    tRef: number,
    tMax: number,
    tFlexMax: number,
    runwayLength: number,
    wind: number,
  ): number {
    if (temp > tFlexMax) {
      return NaN;
    }

    let weightDelta: number;

    if (wind >= 0) {
      const windFactors = A380482xTakeoffPerformanceCalculator.vmcgHeadWindFactor[conf];

      weightDelta = 1000 * (runwayLength * windFactors[0] + windFactors[1]) * wind;
      if (temp > isaTemp) {
        weightDelta +=
          1000 * (runwayLength * windFactors[2] + windFactors[3]) * wind * (Math.min(temp, tRef) - isaTemp);
      }
      if (temp > tRef) {
        weightDelta += 1000 * (runwayLength * windFactors[4] + windFactors[5]) * wind * (Math.min(temp, tMax) - tRef);
      }
      if (temp >= tMax) {
        weightDelta += 1000 * (runwayLength * windFactors[6] + windFactors[7]) * wind * (temp - tMax);
      }
    } else {
      const windFactors = A380482xTakeoffPerformanceCalculator.vmcgTailWindFactor[conf];

      weightDelta = 1000 * (runwayLength * windFactors[0] + windFactors[1]) * wind;
      if (temp > isaTemp) {
        weightDelta +=
          1000 * (runwayLength * windFactors[2] + windFactors[3]) * wind * (Math.min(temp, tRef) - isaTemp);
      }
      if (temp > tRef) {
        weightDelta += 1000 * windFactors[4] * wind * (Math.min(temp, tMax) - tRef);
      }
      if (temp > tMax) {
        weightDelta += 1000 * windFactors[5] * wind * (temp - tMax);
      }
    }

    // cover an edge case near the ends of the data
    if (Math.sign(weightDelta) === Math.sign(wind)) {
      return 0;
    }
    return weightDelta;
  }

  private calculateStabTrim(cg: number): number {
    return MathUtils.round(MathUtils.lerp(cg, 17, 40, 3.8, -2.5, true, true), 0.1);
  }

  /** @inheritdoc */
  public isCgWithinLimits(cg: number, tow: number): boolean {
    const cgLimits = A380482xTakeoffPerformanceCalculator.takeoffCgLimits.get(
      A380482xTakeoffPerformanceCalculator.vec2Cache,
      tow,
    );
    return cg >= cgLimits[0] && cg <= cgLimits[1];
  }
}
