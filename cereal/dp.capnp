using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("ai.comma.openpilot.cereal");
$Java.outerClassname("dp");

@0xbfa7e645486440c7;

# dp.capnp: a home for deprecated structs

# dp
struct DragonConf {
  dpThermalStarted @0 :Bool;
  dpThermalOverheat @1 :Bool;
  dpAtl @2 :Bool;
  dpDashcamd @3 :Bool;
  dpAutoShutdown @4 :Bool;
  dpAthenad @5 :Bool;
  dpUploader @6 :Bool;
  dpLateralMode @7 :UInt8;
  dpSignalOffDelay @8 :Float32;
  dpLcMinMph @9 :UInt8;
  dpLcAutoCont @10 :Bool;
  dpLcAutoMinMph @11 :UInt8;
  dpLcAutoDelay @12 :Float32;
  dpAllowGas @13 :Bool;
  dpFollowingProfileCtrl @14 :Bool;
  dpFollowingProfile @15 :UInt8;
  dpAccelProfileCtrl @16 :Bool;
  dpAccelProfile @17 :UInt8;
  dpGearCheck @18 :Bool;
  dpSpeedCheck @19 :Bool;
  dpUiDisplayMode @20 :UInt8;
  dpUiSpeed @21 :Bool;
  dpUiEvent @22 :Bool;
  dpUiMaxSpeed @23 :Bool;
  dpUiFace @24 :Bool;
  dpUiLane @25 :Bool;
  dpUiLead @26 :Bool;
  dpUiDev @27 :Bool;
  dpUiDevMini @28 :Bool;
  dpUiBlinker @29 :Bool;
  dpUiBrightness @30 :UInt8;
  dpUiVolume @31 :Int8;
  dpToyotaLdw @32 :Bool;
  dpToyotaSng @33 :Bool;
  dpVwTimebombAssist @34 :Bool;
  dpIpAddr @35 :Text;
  dpCameraOffset @36 :Int8;
  dpPathOffset @37 :Int8;
  dpLocale @38 :Text;
  dpSrLearner @39 :Bool;
  dpSrCustom @40 :Float32;
  dpAppd @41 :Bool;
  dpDebug @42 :Bool;
}