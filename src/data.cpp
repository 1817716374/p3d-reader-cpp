#include "internal.hpp"
namespace p3d {
const Json &bgfb_schema() {
    static const Json value = Json::parse(R"p3d({
  "source": "https://github.com/iTwin/imodel-native/blob/main/iModelCore/GeomLibs/serialization/src/FlatBuffer/allcg.fbs",
  "source_sha256": "91ceb846343febd7ce022685d642910e9dbee78e459f85f43d10e689f0735086",
  "license": "Apache-2.0",
  "types": {
    "DPoint3d": {
      "kind": "struct",
      "fields": [
        {
          "name": "x",
          "type": "double"
        },
        {
          "name": "y",
          "type": "double"
        },
        {
          "name": "z",
          "type": "double"
        }
      ]
    },
    "DRay3d": {
      "kind": "struct",
      "fields": [
        {
          "name": "x",
          "type": "double"
        },
        {
          "name": "y",
          "type": "double"
        },
        {
          "name": "z",
          "type": "double"
        },
        {
          "name": "ux",
          "type": "double"
        },
        {
          "name": "uy",
          "type": "double"
        },
        {
          "name": "uz",
          "type": "double"
        }
      ]
    },
    "DPoint2d": {
      "kind": "struct",
      "fields": [
        {
          "name": "x",
          "type": "double"
        },
        {
          "name": "y",
          "type": "double"
        }
      ]
    },
    "DVector3d": {
      "kind": "struct",
      "fields": [
        {
          "name": "x",
          "type": "double"
        },
        {
          "name": "y",
          "type": "double"
        },
        {
          "name": "z",
          "type": "double"
        }
      ]
    },
    "Angle": {
      "kind": "struct",
      "fields": [
        {
          "name": "degrees",
          "type": "double"
        }
      ]
    },
    "DEllipse3d": {
      "kind": "struct",
      "fields": [
        {
          "name": "centerX",
          "type": "double"
        },
        {
          "name": "centerY",
          "type": "double"
        },
        {
          "name": "centerZ",
          "type": "double"
        },
        {
          "name": "vector0X",
          "type": "double"
        },
        {
          "name": "vector0Y",
          "type": "double"
        },
        {
          "name": "vector0Z",
          "type": "double"
        },
        {
          "name": "vector90X",
          "type": "double"
        },
        {
          "name": "vector90Y",
          "type": "double"
        },
        {
          "name": "vector90Z",
          "type": "double"
        },
        {
          "name": "startRadians",
          "type": "double"
        },
        {
          "name": "sweepRadians",
          "type": "double"
        }
      ]
    },
    "DSegment3d": {
      "kind": "struct",
      "fields": [
        {
          "name": "point0X",
          "type": "double"
        },
        {
          "name": "point0Y",
          "type": "double"
        },
        {
          "name": "point0Z",
          "type": "double"
        },
        {
          "name": "point1X",
          "type": "double"
        },
        {
          "name": "point1Y",
          "type": "double"
        },
        {
          "name": "point1Z",
          "type": "double"
        }
      ]
    },
    "DTransform3d": {
      "kind": "struct",
      "fields": [
        {
          "name": "axx",
          "type": "double"
        },
        {
          "name": "axy",
          "type": "double"
        },
        {
          "name": "axz",
          "type": "double"
        },
        {
          "name": "axw",
          "type": "double"
        },
        {
          "name": "ayx",
          "type": "double"
        },
        {
          "name": "ayy",
          "type": "double"
        },
        {
          "name": "ayz",
          "type": "double"
        },
        {
          "name": "ayw",
          "type": "double"
        },
        {
          "name": "azx",
          "type": "double"
        },
        {
          "name": "azy",
          "type": "double"
        },
        {
          "name": "azz",
          "type": "double"
        },
        {
          "name": "azw",
          "type": "double"
        }
      ]
    },
    "DgnBoxDetail": {
      "kind": "struct",
      "fields": [
        {
          "name": "baseOriginX",
          "type": "double"
        },
        {
          "name": "baseOriginY",
          "type": "double"
        },
        {
          "name": "baseOriginZ",
          "type": "double"
        },
        {
          "name": "topOriginX",
          "type": "double"
        },
        {
          "name": "topOriginY",
          "type": "double"
        },
        {
          "name": "topOriginZ",
          "type": "double"
        },
        {
          "name": "vectorXX",
          "type": "double"
        },
        {
          "name": "vectorXY",
          "type": "double"
        },
        {
          "name": "vectorXZ",
          "type": "double"
        },
        {
          "name": "vectorYX",
          "type": "double"
        },
        {
          "name": "vectorYY",
          "type": "double"
        },
        {
          "name": "vectorYZ",
          "type": "double"
        },
        {
          "name": "baseX",
          "type": "double"
        },
        {
          "name": "baseY",
          "type": "double"
        },
        {
          "name": "topX",
          "type": "double"
        },
        {
          "name": "topY",
          "type": "double"
        },
        {
          "name": "capped",
          "type": "bool"
        }
      ]
    },
    "DgnSphereDetail": {
      "kind": "struct",
      "fields": [
        {
          "name": "localToWorld",
          "type": "DTransform3d"
        },
        {
          "name": "startLatitudeRadians",
          "type": "double"
        },
        {
          "name": "latitudeSweepRadians",
          "type": "double"
        },
        {
          "name": "capped",
          "type": "bool"
        }
      ]
    },
    "DgnConeDetail": {
      "kind": "struct",
      "fields": [
        {
          "name": "centerAX",
          "type": "double"
        },
        {
          "name": "centerAY",
          "type": "double"
        },
        {
          "name": "centerAZ",
          "type": "double"
        },
        {
          "name": "centerBX",
          "type": "double"
        },
        {
          "name": "centerBY",
          "type": "double"
        },
        {
          "name": "centerBZ",
          "type": "double"
        },
        {
          "name": "vector0X",
          "type": "double"
        },
        {
          "name": "vector0Y",
          "type": "double"
        },
        {
          "name": "vector0Z",
          "type": "double"
        },
        {
          "name": "vector90X",
          "type": "double"
        },
        {
          "name": "vector90Y",
          "type": "double"
        },
        {
          "name": "vector90Z",
          "type": "double"
        },
        {
          "name": "radiusA",
          "type": "double"
        },
        {
          "name": "radiusB",
          "type": "double"
        },
        {
          "name": "capped",
          "type": "bool"
        }
      ]
    },
    "DgnTorusPipeDetail": {
      "kind": "struct",
      "fields": [
        {
          "name": "centerX",
          "type": "double"
        },
        {
          "name": "centerY",
          "type": "double"
        },
        {
          "name": "centerZ",
          "type": "double"
        },
        {
          "name": "vectorXX",
          "type": "double"
        },
        {
          "name": "vectorXY",
          "type": "double"
        },
        {
          "name": "vectorXZ",
          "type": "double"
        },
        {
          "name": "vectorYX",
          "type": "double"
        },
        {
          "name": "vectorYY",
          "type": "double"
        },
        {
          "name": "vectorYZ",
          "type": "double"
        },
        {
          "name": "majorRadius",
          "type": "double"
        },
        {
          "name": "minorRadius",
          "type": "double"
        },
        {
          "name": "sweepRadians",
          "type": "double"
        },
        {
          "name": "capped",
          "type": "bool"
        }
      ]
    },
    "LineSegment": {
      "kind": "table",
      "fields": [
        {
          "name": "segment",
          "type": "DSegment3d"
        }
      ]
    },
    "LineString": {
      "kind": "table",
      "fields": [
        {
          "name": "points",
          "type": "[double]"
        }
      ]
    },
    "PointString": {
      "kind": "table",
      "fields": [
        {
          "name": "points",
          "type": "[double]"
        }
      ]
    },
    "EllipticArc": {
      "kind": "table",
      "fields": [
        {
          "name": "arc",
          "type": "DEllipse3d"
        }
      ]
    },
    "BsplineCurve": {
      "kind": "table",
      "fields": [
        {
          "name": "order",
          "type": "int"
        },
        {
          "name": "closed",
          "type": "bool"
        },
        {
          "name": "poles",
          "type": "[double]"
        },
        {
          "name": "weights",
          "type": "[double]"
        },
        {
          "name": "knots",
          "type": "[double]"
        }
      ]
    },
    "InterpolationCurve": {
      "kind": "table",
      "fields": [
        {
          "name": "order",
          "type": "int"
        },
        {
          "name": "closed",
          "type": "bool"
        },
        {
          "name": "isChordLenKnots",
          "type": "int"
        },
        {
          "name": "isColinearTangents",
          "type": "int"
        },
        {
          "name": "isChordLenTangents",
          "type": "int"
        },
        {
          "name": "isNaturalTangents",
          "type": "int"
        },
        {
          "name": "startTangent",
          "type": "DVector3d"
        },
        {
          "name": "endTangent",
          "type": "DVector3d"
        },
        {
          "name": "fitPoints",
          "type": "[double]"
        },
        {
          "name": "knots",
          "type": "[double]"
        }
      ]
    },
    "AkimaCurve": {
      "kind": "table",
      "fields": [
        {
          "name": "points",
          "type": "[double]"
        }
      ]
    },
    "CatenaryCurve": {
      "kind": "table",
      "fields": [
        {
          "name": "a",
          "type": "double"
        },
        {
          "name": "origin",
          "type": "DPoint3d"
        },
        {
          "name": "vectorU",
          "type": "DVector3d"
        },
        {
          "name": "vectorV",
          "type": "DVector3d"
        },
        {
          "name": "x0",
          "type": "double"
        },
        {
          "name": "x1",
          "type": "double"
        }
      ]
    },
    "PartialCurve": {
      "kind": "table",
      "fields": [
        {
          "name": "fraction0",
          "type": "double"
        },
        {
          "name": "fraction1",
          "type": "double"
        },
        {
          "name": "target",
          "type": "VariantGeometry"
        }
      ]
    },
    "CurvePrimitiveId": {
      "kind": "table",
      "fields": [
        {
          "name": "type",
          "type": "short"
        },
        {
          "name": "geomIndex",
          "type": "short"
        },
        {
          "name": "partIndex",
          "type": "short"
        },
        {
          "name": "bytes",
          "type": "[ubyte]"
        }
      ]
    },
    "CurveVector": {
      "kind": "table",
      "fields": [
        {
          "name": "type",
          "type": "int"
        },
        {
          "name": "curves",
          "type": "[VariantGeometry]"
        }
      ]
    },
    "VectorOfVariantGeometry": {
      "kind": "table",
      "fields": [
        {
          "name": "members",
          "type": "[VariantGeometry]"
        }
      ]
    },
    "BsplineSurface": {
      "kind": "table",
      "fields": [
        {
          "name": "poles",
          "type": "[double]"
        },
        {
          "name": "weights",
          "type": "[double]"
        },
        {
          "name": "knotsU",
          "type": "[double]"
        },
        {
          "name": "knotsV",
          "type": "[double]"
        },
        {
          "name": "numPolesU",
          "type": "int"
        },
        {
          "name": "numPolesV",
          "type": "int"
        },
        {
          "name": "orderU",
          "type": "int"
        },
        {
          "name": "orderV",
          "type": "int"
        },
        {
          "name": "numRulesU",
          "type": "int"
        },
        {
          "name": "numRulesV",
          "type": "int"
        },
        {
          "name": "holeOrigin",
          "type": "int"
        },
        {
          "name": "boundaries",
          "type": "CurveVector"
        },
        {
          "name": "closedU",
          "type": "bool"
        },
        {
          "name": "closedV",
          "type": "bool"
        }
      ]
    },
    "DgnBox": {
      "kind": "table",
      "fields": [
        {
          "name": "detail",
          "type": "DgnBoxDetail"
        }
      ]
    },
    "DgnSphere": {
      "kind": "table",
      "fields": [
        {
          "name": "detail",
          "type": "DgnSphereDetail"
        }
      ]
    },
    "DgnCone": {
      "kind": "table",
      "fields": [
        {
          "name": "detail",
          "type": "DgnConeDetail"
        }
      ]
    },
    "DgnTorusPipe": {
      "kind": "table",
      "fields": [
        {
          "name": "detail",
          "type": "DgnTorusPipeDetail"
        }
      ]
    },
    "DgnExtrusion": {
      "kind": "table",
      "fields": [
        {
          "name": "baseCurve",
          "type": "CurveVector"
        },
        {
          "name": "extrusionVector",
          "type": "DVector3d"
        },
        {
          "name": "capped",
          "type": "bool"
        }
      ]
    },
    "DgnRotationalSweep": {
      "kind": "table",
      "fields": [
        {
          "name": "baseCurve",
          "type": "CurveVector"
        },
        {
          "name": "axis",
          "type": "DRay3d"
        },
        {
          "name": "sweepRadians",
          "type": "double"
        },
        {
          "name": "numVRules",
          "type": "int"
        },
        {
          "name": "capped",
          "type": "bool"
        }
      ]
    },
    "DgnRuledSweep": {
      "kind": "table",
      "fields": [
        {
          "name": "curves",
          "type": "[CurveVector]"
        },
        {
          "name": "capped",
          "type": "bool"
        }
      ]
    },
    "PolyfaceAuxChannelData": {
      "kind": "table",
      "fields": [
        {
          "name": "input",
          "type": "double"
        },
        {
          "name": "values",
          "type": "[double]"
        }
      ]
    },
    "PolyfaceAuxChannel": {
      "kind": "table",
      "fields": [
        {
          "name": "dataType",
          "type": "int"
        },
        {
          "name": "name",
          "type": "string"
        },
        {
          "name": "inputName",
          "type": "string"
        },
        {
          "name": "data",
          "type": "[PolyfaceAuxChannelData]"
        }
      ]
    },
    "PolyfaceAuxData": {
      "kind": "table",
      "fields": [
        {
          "name": "indices",
          "type": "[int]"
        },
        {
          "name": "channels",
          "type": "[PolyfaceAuxChannel]"
        }
      ]
    },
    "TaggedNumericData": {
      "kind": "table",
      "fields": [
        {
          "name": "tagA",
          "type": "int"
        },
        {
          "name": "tagB",
          "type": "int"
        },
        {
          "name": "intData",
          "type": "[int]"
        },
        {
          "name": "doubleData",
          "type": "[double]"
        }
      ]
    },
    "Polyface": {
      "kind": "table",
      "fields": [
        {
          "name": "point",
          "type": "[double]"
        },
        {
          "name": "param",
          "type": "[double]"
        },
        {
          "name": "normal",
          "type": "[double]"
        },
        {
          "name": "doubleColor",
          "type": "[double]"
        },
        {
          "name": "intColor",
          "type": "[uint]"
        },
        {
          "name": "pointIndex",
          "type": "[int]"
        },
        {
          "name": "paramIndex",
          "type": "[int]"
        },
        {
          "name": "normalIndex",
          "type": "[int]"
        },
        {
          "name": "colorIndex",
          "type": "[int]"
        },
        {
          "name": "colorTable",
          "type": "[int]"
        },
        {
          "name": "numPerFace",
          "type": "int"
        },
        {
          "name": "numPerRow",
          "type": "int"
        },
        {
          "name": "meshStyle",
          "type": "int"
        },
        {
          "name": "twoSided",
          "type": "bool"
        },
        {
          "name": "faceIndex",
          "type": "[int]"
        },
        {
          "name": "faceData",
          "type": "[double]"
        },
        {
          "name": "auxData",
          "type": "PolyfaceAuxData"
        },
        {
          "name": "expectedClosure",
          "type": "int"
        },
        {
          "name": "taggedNumericData",
          "type": "TaggedNumericData"
        },
        {
          "name": "edgeMateIndex",
          "type": "[int]"
        }
      ]
    },
    "TransitionSpiralDetail": {
      "kind": "struct",
      "fields": [
        {
          "name": "transform",
          "type": "DTransform3d"
        },
        {
          "name": "fractionA",
          "type": "double"
        },
        {
          "name": "fractionB",
          "type": "double"
        },
        {
          "name": "bearing0Radians",
          "type": "double"
        },
        {
          "name": "bearing1Radians",
          "type": "double"
        },
        {
          "name": "curvature0",
          "type": "double"
        },
        {
          "name": "curvature1",
          "type": "double"
        },
        {
          "name": "spiralType",
          "type": "int"
        },
        {
          "name": "constructionHint",
          "type": "int"
        }
      ]
    },
    "DirectSpiralDetail": {
      "kind": "struct",
      "fields": [
        {
          "name": "nominalLength",
          "type": "double"
        },
        {
          "name": "trueLength",
          "type": "double"
        },
        {
          "name": "doubleTag0",
          "type": "double"
        },
        {
          "name": "doubleTag1",
          "type": "double"
        },
        {
          "name": "intTag0",
          "type": "double"
        },
        {
          "name": "intTag1",
          "type": "double"
        }
      ]
    },
    "TransitionSpiral": {
      "kind": "table",
      "fields": [
        {
          "name": "detail",
          "type": "TransitionSpiralDetail"
        },
        {
          "name": "extraData",
          "type": "[double]"
        },
        {
          "name": "directDetail",
          "type": "DirectSpiralDetail"
        }
      ]
    },
    "VariantGeometryUnion": {
      "kind": "union",
      "fields": {
        "1": "LineSegment",
        "2": "EllipticArc",
        "3": "BsplineCurve",
        "4": "LineString",
        "5": "CurveVector",
        "6": "DgnCone",
        "7": "DgnSphere",
        "8": "DgnTorusPipe",
        "9": "DgnBox",
        "10": "DgnExtrusion",
        "11": "DgnRotationalSweep",
        "12": "DgnRuledSweep",
        "13": "Polyface",
        "14": "BsplineSurface",
        "15": "VectorOfVariantGeometry",
        "16": "InterpolationCurve",
        "17": "TransitionSpiral",
        "18": "PointString",
        "19": "AkimaCurve",
        "20": "CatenaryCurve",
        "21": "PartialCurve"
      }
    },
    "VariantGeometry": {
      "kind": "table",
      "fields": [
        {
          "name": "geometry",
          "type": "VariantGeometryUnion"
        },
        {
          "name": "tag",
          "type": "CurvePrimitiveId"
        }
      ]
    }
  }
})p3d");
    return value;
}
const Json &default_palette() {
    static const Json value = Json::parse(
        R"p3d({"source":"https://raw.githubusercontent.com/OSGeo/gdal/master/ogr/ogrsf_frmts/dgn/dgnhelp.cpp","rgb":[[255,255,255],[0,0,255],[0,255,0],[255,0,0],[255,255,0],[255,0,255],[255,127,0],[0,255,255],[64,64,64],[192,192,192],[254,0,96],[160,224,0],[0,254,160],[128,0,160],[176,176,176],[0,240,240],[240,240,240],[0,0,240],[0,240,0],[240,0,0],[240,240,0],[240,0,240],[240,122,0],[0,240,240],[240,240,240],[0,0,240],[0,240,0],[240,0,0],[240,240,0],[240,0,240],[240,122,0],[0,225,225],[225,225,225],[0,0,225],[0,225,0],[225,0,0],[225,225,0],[225,0,225],[225,117,0],[0,225,225],[225,225,225],[0,0,225],[0,225,0],[225,0,0],[225,225,0],[225,0,225],[225,117,0],[0,210,210],[210,210,210],[0,0,210],[0,210,0],[210,0,0],[210,210,0],[210,0,210],[210,112,0],[0,210,210],[210,210,210],[0,0,210],[0,210,0],[210,0,0],[210,210,0],[210,0,210],[210,112,0],[0,195,195],[195,195,195],[0,0,195],[0,195,0],[195,0,0],[195,195,0],[195,0,195],[195,107,0],[0,195,195],[195,195,195],[0,0,195],[0,195,0],[195,0,0],[195,195,0],[195,0,195],[195,107,0],[0,180,180],[180,180,180],[0,0,180],[0,180,0],[180,0,0],[180,180,0],[180,0,180],[180,102,0],[0,180,180],[180,180,180],[0,0,180],[0,180,0],[180,0,0],[180,180,0],[180,0,180],[180,102,0],[0,165,165],[165,165,165],[0,0,165],[0,165,0],[165,0,0],[165,165,0],[165,0,165],[165,97,0],[0,165,165],[165,165,165],[0,0,165],[0,165,0],[165,0,0],[165,165,0],[165,0,165],[165,97,0],[0,150,150],[150,150,150],[0,0,150],[0,150,0],[150,0,0],[150,150,0],[150,0,150],[150,92,0],[0,150,150],[150,150,150],[0,0,150],[0,150,0],[150,0,0],[150,150,0],[150,0,150],[150,92,0],[0,135,135],[135,135,135],[0,0,135],[0,135,0],[135,0,0],[135,135,0],[135,0,135],[135,87,0],[0,135,135],[135,135,135],[0,0,135],[0,135,0],[135,0,0],[135,135,0],[135,0,135],[135,87,0],[0,120,120],[120,120,120],[0,0,120],[0,120,0],[120,0,0],[120,120,0],[120,0,120],[120,82,0],[0,120,120],[120,120,120],[0,0,120],[0,120,0],[120,0,0],[120,120,0],[120,0,120],[120,82,0],[0,105,105],[105,105,105],[0,0,105],[0,105,0],[105,0,0],[105,105,0],[105,0,105],[105,77,0],[0,105,105],[105,105,105],[0,0,105],[0,105,0],[105,0,0],[105,105,0],[105,0,105],[105,77,0],[0,90,90],[90,90,90],[0,0,90],[0,90,0],[90,0,0],[90,90,0],[90,0,90],[90,72,0],[0,90,90],[90,90,90],[0,0,90],[0,90,0],[90,0,0],[90,90,0],[90,0,90],[90,72,0],[0,75,75],[75,75,75],[0,0,75],[0,75,0],[75,0,0],[75,75,0],[75,0,75],[75,67,0],[0,75,75],[75,75,75],[0,0,75],[0,75,0],[75,0,0],[75,75,0],[75,0,75],[75,67,0],[0,60,60],[60,60,60],[0,0,60],[0,60,0],[60,0,0],[60,60,0],[60,0,60],[60,62,0],[0,60,60],[60,60,60],[0,0,60],[0,60,0],[60,0,0],[60,60,0],[60,0,60],[60,62,0],[0,45,45],[45,45,45],[0,0,45],[0,45,0],[45,0,0],[45,45,0],[45,0,45],[45,57,0],[0,45,45],[45,45,45],[0,0,45],[0,45,0],[45,0,0],[45,45,0],[45,0,45],[45,57,0],[0,30,30],[30,30,30],[0,0,30],[0,30,0],[30,0,0],[30,30,0],[30,0,30],[30,52,0],[0,30,30],[30,30,30],[0,0,30],[0,30,0],[30,0,0],[30,30,0],[30,0,30],[192,192,192],[28,0,100]]})p3d");
    return value;
}
} // namespace p3d
