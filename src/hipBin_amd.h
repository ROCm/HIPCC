/*
Copyright (c) 2021 Advanced Micro Devices, Inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef SRC_HIPBIN_AMD_H_
#define SRC_HIPBIN_AMD_H_

#include "hipBin_base.h"
#include "hipBin_util.h"
#include "json.hpp"
#include <vector>
#include <set>
#include <string>
#include <unordered_set>
#include <cassert>

#include <fstream>
#include <sstream>
#include <chrono>
#include <sys/stat.h>
#include <sys/file.h>
#include <filesystem>
#include <fcntl.h>

// Use (void) to silent unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

// Known Features
 std::unordered_set
 <std::string> knownFeatures =  { "sramecc-" , "sramecc+",
                                  "xnack-", "xnack+" };

struct HipGpuBinaryCache {
  const size_t       CACHE_SIZE_LIMIT = 96000000; //0x400000000; // 16 Gb
  const size_t       SRC_MIN_SIZE     = 0; //0x800;
  const size_t       SRC_SIZE_LIMIT   = 0x100000;
  const size_t       CUI_SIZE_LIMIT   = 0x800000;
  const int          MAX_SUBDIR_NUM   = 9999;
  const uint32_t     ENTRY_EST_SIZE   = 0x500000; // 5 Mb
  const uint32_t     DATA_FILES_MASK  = 0x3f; // 64 data files.
  const std::string  CACHE_DIR_SUFFIX     = ".cache/hip";
  const std::string  CACHE_DESC_FILE_NAME = "cache.json";

  struct {
    size_t       size;
    size_t       modTime;
    std::string  path;
    std::string  name;
  } srcFileInfo;

  struct ENTRY_INFO {
    enum {
      FOUND_CACHED,
      CREATED_NEW_SUBDIR,
      USED_EMPTY_SUBDIR,
      EVICTED_OLD_ENTRY
    } howAdded;

    std::string     subDir;
    std::string     cachedHipfbName;
    std::string     dataFileName;
    //nlohmann::json  jEntry;
    int             subDirNum;
  } cacheEntryInfo;

  class FileLock {
    int f;
  public:
    FileLock(const std::string& path, bool excl = false) {
      if ((f = open(path.c_str(), O_RDONLY)) != -1) { flock(f, excl ? LOCK_EX : LOCK_SH); }
    }
    ~FileLock() { release(); }
    void release() { 
      if (f != -1) { flock(f, LOCK_UN); close(f); f = -1; }
    }
  };

  bool  enabled = false;
  bool  verbose = false;
  bool  multTargets = false;
  std::string     cacheDir;
  std::string     hipVersion;
  std::string     clangPath;
  std::string     clangArgs;
  nlohmann::json  cacheDesc;

  HipGpuBinaryCache(const std::string& hipVer, bool verb); // : hipVersion(hipVer), verbose(verb);
  bool findCachedHipfb(std::string& cmd);
  bool findAvailableEntry();
  bool preprocessSource(const std::string& srcFileName, std::stringstream& prepText);
  bool preprocessSourceToFile(const std::string& cmd);
  void addSaveHipfbOption(std::string& cmd);
  void addUseHipfbOption(std::string& cmd);
  bool readCachedPrepSource(const nlohmann::json& jentry, std::stringstream& text);
  int  evictOldEntries();
  int  evictEntry(std::pair<uint32_t, uint32_t>& entry, size_t& totalSize);
  bool updateCacheDescFile(bool releaseLock = true);
  bool updateFilesPostBuild(bool buildFailed = false);
  bool updateDataFilesPostBuild(size_t subdirSize, uint32_t& entryID);
  bool updateCacheDescFilePostBuild(size_t subdirSize, uint32_t entryID, bool buildFailed = false);
  bool createCacheSubDir();
  bool cleanUpCache();
  bool initCache();
  bool initCacheDescFile();
  std::string getSubDirName(int subDirNum);
};


class HipBinAmd : public HipBinBase {
 private:
  HipBinUtil* hipBinUtilPtr_;
  string hipClangPath_ = "";
  string roccmPathEnv_, hipRocclrPathEnv_, hsaPathEnv_;
  PlatformInfo platformInfoAMD_;
  string hipCFlags_, hipCXXFlags_, hipLdFlags_;

  void constructRocclrHomePath();
  void constructHsaPath();

 public:
  HipBinAmd();
  virtual ~HipBinAmd() = default;
  virtual bool detectPlatform();
  virtual void constructCompilerPath();
  virtual const string& getCompilerPath() const;
  virtual const PlatformInfo& getPlatformInfo() const;
  virtual string getCppConfig();
  virtual void printFull();
  virtual void printCompilerInfo() const;
  virtual string getCompilerVersion();
  virtual void checkHipconfig();
  virtual string getDeviceLibPath() const;
  virtual string getHipLibPath() const;
  virtual string getHipCC() const;
  virtual string getCompilerIncludePath();
  virtual string getHipInclude() const;
  virtual void initializeHipCXXFlags();
  virtual void initializeHipCFlags();
  virtual void initializeHipLdFlags();
  virtual const string& getHipCXXFlags() const;
  virtual const string& getHipCFlags() const;
  virtual const string& getHipLdFlags() const;
  virtual void executeHipCCCmd(vector<string> argv);
  // non virtual functions
  const string& getHsaPath() const;
  const string& getRocclrHomePath() const;
};

HipBinAmd::HipBinAmd() {
  PlatformInfo platformInfo;
  platformInfo.os = getOSInfo();
  platformInfo.platform = amd;
  platformInfo.runtime = rocclr;
  platformInfo.compiler = clang;
  platformInfoAMD_ = platformInfo;
  constructRocclrHomePath();    // constructs RocclrHomePath
  constructHsaPath();           // constructs hsa path
  constructCompilerPath();
}

// returns the Rocclr Home path
void HipBinAmd::constructRocclrHomePath() {
  fs::path full_path(fs::current_path());
  fs::path hipvars_dir = full_path;
  fs::path bitcode = hipvars_dir;
  string rocclrHomePath = getEnvVariables().hipRocclrPathEnv_;
  if (rocclrHomePath.empty()) {
    bitcode /= "../lib/bitcode";
    if (!fs::exists(bitcode)) {
      rocclrHomePath = getHipPath();
    } else {
      hipvars_dir /= "..";
      rocclrHomePath = hipvars_dir.string();
    }
  }
  hipRocclrPathEnv_ = rocclrHomePath;
}


// construct hsa Path
void HipBinAmd::constructHsaPath() {
  fs::path hsaPathfs;
  string hsaPath = getEnvVariables().hsaPathEnv_;
  if (hsaPath.empty()) {
    hsaPath = getRoccmPath();
    hsaPathfs = hsaPath;
    hsaPathfs /= "hsa";
    hsaPath = hsaPathfs.string();
    hsaPathEnv_ = hsaPath;
  } else {
    hsaPathEnv_ = hsaPath;
  }
}

// returns the Rocclr Home path
const string& HipBinAmd::getRocclrHomePath() const {
  return hipRocclrPathEnv_;
}

// returns hsa Path
const string& HipBinAmd::getHsaPath() const {
  // return variables_.hsaPathEnv_;
  return hsaPathEnv_;
}


const string& HipBinAmd::getHipCFlags() const {
  return hipCFlags_;
}


const string& HipBinAmd::getHipLdFlags() const {
  return hipLdFlags_;
}


void HipBinAmd::initializeHipLdFlags() {
  string hipLibPath;
  string hipLdFlags;
  const string& hipClangPath = getCompilerPath();
  // If $HIPCC clang++ is not compiled, use clang instead
  string hipCC = "\"" + hipClangPath + "/clang++";
  if (!fs::exists(hipCC)) {
    hipLdFlags = "--driver-mode=g++";
  }
  hipLibPath = getHipLibPath();
  hipLdFlags += " -L\"" + hipLibPath + "\"";
  const OsType& os = getOSInfo();
  if (os == windows) {
    hipLdFlags += " -lamdhip64";
  }
  hipLdFlags_ = hipLdFlags;
}

void HipBinAmd::initializeHipCFlags() {
  string hipCFlags;
  string hipclangIncludePath;
  hipclangIncludePath = getHipInclude();
  hipCFlags += " -isystem \"" + hipclangIncludePath + "\"";
  const OsType& os = getOSInfo();
  if (os != windows) {
    string hsaPath;
    hsaPath = getHsaPath();
    hipCFlags += " -isystem " + hsaPath + "/include";
  }
  string hipIncludePath;
  hipIncludePath = getHipInclude();
  hipCFlags += " -isystem \"" + hipIncludePath + "\"";
  hipCFlags_ = hipCFlags;
}

const string& HipBinAmd::getHipCXXFlags() const {
  return hipCXXFlags_;
}


string HipBinAmd::getHipInclude() const {
  const string& rocclrHomePath = getRocclrHomePath();
  fs::path hipIncludefs = rocclrHomePath;
  hipIncludefs /= "include";
  if (hipIncludefs.string().empty()) {
    const string& hipPath = getHipPath();
    hipIncludefs = hipPath;
    hipIncludefs /= "include";
  }
  string hipInclude = hipIncludefs.string();
  return hipInclude;
}


void HipBinAmd::initializeHipCXXFlags() {
  string hipCXXFlags;
  const OsType& os = getOSInfo();
  string hipClangIncludePath;
  hipClangIncludePath = getCompilerIncludePath();
  hipCXXFlags += " -isystem \"" + hipClangIncludePath;
  fs::path hipCXXFlagsTempFs = hipCXXFlags;
  hipCXXFlagsTempFs /= "..\"";
  hipCXXFlags = hipCXXFlagsTempFs.string();
  const EnvVariables& var = getEnvVariables();
  // Allow __fp16 as function parameter and return type.
  if (var.hipClangHccCompactModeEnv_.compare("1") == 0) {
    hipCXXFlags +=
    " -Xclang -fallow-half-arguments-and-returns -D__HIP_HCC_COMPAT_MODE__=1";
  }

  if (os != windows) {
    const string& hsaPath = getHsaPath();
    hipCXXFlags += " -isystem " + hsaPath + "/include";
  }
  // Add paths to common HIP includes:
  string hipIncludePath;
  hipIncludePath = getHipInclude();
  hipCXXFlags += " -isystem \"" + hipIncludePath + "\"";
  hipCXXFlags_ = hipCXXFlags;
}

// populates clang path.
void HipBinAmd::constructCompilerPath() {
  string complierPath;
  const EnvVariables& envVariables = getEnvVariables();
  if (envVariables.hipClangPathEnv_.empty()) {
    fs::path hipClangPath;
    const OsType& osInfo = getOSInfo();
    if (osInfo == windows) {
      complierPath = getHipPath();
      hipClangPath = complierPath;
      hipClangPath /= "bin";
    } else {
      complierPath = getRoccmPath();
      hipClangPath = complierPath;
      hipClangPath /= "llvm/bin";
    }
    complierPath = hipClangPath.string();
  } else {
    complierPath = envVariables.hipClangPathEnv_;
  }
  hipClangPath_ = complierPath;
}




// returns clang path.
const string& HipBinAmd::getCompilerPath() const {
  return hipClangPath_;
}

void HipBinAmd::printCompilerInfo() const {
  const OsType& os = getOSInfo();
  const string& hipClangPath = getCompilerPath();
  const string& hipPath = getHipPath();
  if (os == windows) {
    string cmd = hipClangPath + "/clang++ --version";
    system(cmd.c_str());  // hipclang version
    cout << "llc-version :" << endl;
    cmd = hipClangPath + "/llc --version";
    system(cmd.c_str());  // llc version
    cout << "hip-clang-cxxflags :" << endl;
    cmd = hipPath + "/bin/hipcc  --cxxflags";
    system(cmd.c_str());  // cxx flags
    cout << endl << "hip-clang-ldflags :" << endl;
    cmd = hipPath + "/bin/hipcc --ldflags";
    system(cmd.c_str());  // ld flags
    cout << endl;
  } else {
    string cmd = hipClangPath + "/clang++ --version";
    system(cmd.c_str());  // hipclang version
    cmd = hipClangPath + "/llc --version";
    system(cmd.c_str());  // llc version
    cout << "hip-clang-cxxflags :" << endl;
    cmd = hipPath + "/bin/hipcc --cxxflags";
    system(cmd.c_str());  // cxx flags
    cout << endl << "hip-clang-ldflags :" << endl;
    cmd = hipPath + "/bin/hipcc --ldflags";
    system(cmd.c_str());  // ldflags version
    cout << endl;
  }
}

string HipBinAmd::getCompilerVersion() {
  string out, complierVersion;
  const string& hipClangPath = getCompilerPath();
  fs::path cmdAmd = hipClangPath;
  cmdAmd /= "clang++";
  if (canRunCompiler(cmdAmd.string(), out) || canRunCompiler("clang++", out)) {
    regex regexp("([0-9.]+)");
    smatch m;
    if (regex_search(out, m, regexp)) {
      if (m.size() > 1) {
        // get the index =1 match, 0=whole match we ignore
        std::ssub_match sub_match = m[1];
        complierVersion = sub_match.str();
      }
    }
  } else {
    cout << "Hip Clang Compiler not found" << endl;
  }
  return complierVersion;
}



const PlatformInfo& HipBinAmd::getPlatformInfo() const {
  return platformInfoAMD_;
}


string HipBinAmd::getCppConfig() {
  string cppConfig = " -D__HIP_PLATFORM_HCC__= -D__HIP_PLATFORM_AMD__=";

  string compilerVersion;
  compilerVersion = getCompilerVersion();

  fs::path hipPathInclude, hipClangInclude, cppConfigFs;
  string hipClangVersionPath;
  const string& hipPath = getHipPath();
  hipPathInclude = hipPath;
  hipPathInclude /= "include";

  const string& compilerPath = getCompilerPath();
  hipClangInclude = compilerPath;
  hipClangInclude = hipClangInclude.parent_path();
  hipClangInclude /= "lib/clang/";
  hipClangInclude /= compilerVersion;
  string hipClangPath = hipClangInclude.string();

  const OsType& osInfo = getOSInfo();
  if (osInfo == windows) {
    cppConfig += " -I" + hipPathInclude.string() + " -I" + hipClangPath;
    cppConfigFs = cppConfig;
    cppConfigFs /= "/";
  } else {
    const string& hsaPath = getHsaPath();
    cppConfig += " -I" + hipPathInclude.string() +
                 " -I" + hipClangPath + " -I" + hsaPath;
    cppConfigFs = cppConfig;
    cppConfigFs /= "include";
    cppConfig = cppConfigFs.string();
  }
  return cppConfig;
}

string HipBinAmd::getDeviceLibPath() const {
  const EnvVariables& var = getEnvVariables();
  const string& rocclrHomePath = getRocclrHomePath();
  const string& roccmPath = getRoccmPath();
  fs::path bitCodePath = rocclrHomePath;
  bitCodePath /= "lib/bitcode";
  string deviceLibPath = var.deviceLibPathEnv_;
  if (deviceLibPath.empty() && fs::exists(bitCodePath)) {
    deviceLibPath = bitCodePath.string();
  }

  if (deviceLibPath.empty()) {
    fs::path amdgcnBitcode = roccmPath;
    amdgcnBitcode /= "amdgcn/bitcode";
    if (fs::exists(amdgcnBitcode)) {
      deviceLibPath = amdgcnBitcode.string();
    } else {
      // This path is to support an older build of the device library
      // TODO(hipcc): To be removed in the future.
      fs::path lib = roccmPath;
      lib /= "lib";
      deviceLibPath = lib.string();
    }
  }
  return deviceLibPath;
}


bool HipBinAmd::detectPlatform() {
  string out;
  const string& hipClangPath = getCompilerPath();
  fs::path cmdAmd = hipClangPath;
  cmdAmd /= "clang++";
  const EnvVariables& var = getEnvVariables();
  bool detected = false;
  if (var.hipPlatformEnv_.empty()) {
    if (canRunCompiler(cmdAmd.string(), out) ||
       (canRunCompiler("clang++", out))) {
      detected = true;
    }
  } else {
    if (var.hipPlatformEnv_ == "amd" ||
        var.hipPlatformEnv_ == "hcc") {
      detected = true;
      if (var.hipPlatformEnv_ == "hcc")
        cout <<
        "Warning: HIP_PLATFORM=hcc is deprecated."<<
        "Please use HIP_PLATFORM=amd." << endl;
    }
  }
  return detected;
}



string HipBinAmd::getHipLibPath() const {
  string hipLibPath;
  const EnvVariables& env = getEnvVariables();
  if (env.hipLibPathEnv_.empty()) {
    const string& rocclrHomePath = getRocclrHomePath();
    fs::path libPath = rocclrHomePath;
    libPath /= "lib";
    hipLibPath = libPath.string();
  }
  if (hipLibPath.empty()) {
    const string& hipPath = getHipPath();
    fs::path libPath = hipPath;
    libPath /= "lib";
    hipLibPath = libPath.string();
  }
  return hipLibPath;
}

string HipBinAmd::getHipCC() const {
  string hipCC;
  const string& hipClangPath = getCompilerPath();
  fs::path compiler = hipClangPath;
  compiler /= "clang++";
  if (!fs::exists(compiler)) {
    fs::path compiler = hipClangPath;
    compiler /= "clang";
  }
  hipCC = compiler.string();
  return hipCC;
}



string HipBinAmd::getCompilerIncludePath() {
  string hipClangVersion, includePath, compilerIncludePath;
  const string& hipClangPath = getCompilerPath();
  hipClangVersion = getCompilerVersion();
  fs::path includePathfs = hipClangPath;
  includePathfs = includePathfs.parent_path();
  includePathfs /= "lib/clang/";
  includePathfs /= hipClangVersion;
  includePathfs /= "include";
  includePathfs = fs::absolute(includePathfs).string();
  compilerIncludePath = includePathfs.string();
  return compilerIncludePath;
}


void HipBinAmd::checkHipconfig() {
  printFull();
  cout << endl << "Check system installation: " << endl;
  cout << "check hipconfig in PATH..." << endl;
  if (system("which hipconfig > /dev/null 2>&1") != 0) {
    cout << "FAIL " << endl;
  } else {
    cout << "good" << endl;
  }
  string ldLibraryPath;
  const EnvVariables& env = getEnvVariables();
  ldLibraryPath = env.ldLibraryPathEnv_;
  const string& hsaPath = getHsaPath();
  cout << "check LD_LIBRARY_PATH (" << ldLibraryPath <<
          ") contains HSA_PATH (" << hsaPath << ")..." << endl;
  if (ldLibraryPath.find(hsaPath) == string::npos) {
    cout << "FAIL" << endl;
  } else {
    cout << "good" << endl;
  }
}

void HipBinAmd::printFull() {
  const string& hipVersion = getHipVersion();
  const string& hipPath = getHipPath();
  const string& roccmPath = getRoccmPath();
  const PlatformInfo& platformInfo = getPlatformInfo();
  const string& ccpConfig = getCppConfig();
  const string& hsaPath = getHsaPath();
  const string& hipClangPath = getCompilerPath();

  cout << "HIP version: " << hipVersion << endl;
  cout << endl << "==hipconfig" << endl;
  cout << "HIP_PATH           :" << hipPath << endl;
  cout << "ROCM_PATH          :" << roccmPath << endl;
  cout << "HIP_COMPILER       :" << CompilerTypeStr(
                                    platformInfo.compiler) << endl;
  cout << "HIP_PLATFORM       :" << PlatformTypeStr(
                                    platformInfo.platform) << endl;
  cout << "HIP_RUNTIME        :" << RuntimeTypeStr(
                                    platformInfo.runtime) << endl;
  cout << "CPP_CONFIG         :" << ccpConfig << endl;

  cout << endl << "==hip-clang" << endl;
  cout << "HSA_PATH           :" << hsaPath << endl;
  cout << "HIP_CLANG_PATH     :" << hipClangPath << endl;
  printCompilerInfo();
  cout << endl << "== Envirnoment Variables" << endl;
  printEnvironmentVariables();
  getSystemInfo();
  if (fs::exists("/usr/bin/lsb_release"))
    system("/usr/bin/lsb_release -a");
  cout << endl;
}


void HipBinAmd::executeHipCCCmd(vector<string> argv) {
  if (argv.size() < 2) {
    cout<< "No Arguments passed, exiting ...\n";
    exit(EXIT_SUCCESS);
  }
  const EnvVariables& var = getEnvVariables();
  int verbose = 0;
  if (!var.verboseEnv_.empty())
    verbose = stoi(var.verboseEnv_);

  // Verbose: 0x1=commands, 0x2=paths, 0x4=hipcc args, 0x8=hipfb cache
  // set if user explicitly requests -stdlib=libc++
  // (else we default to libstdc++ for better interop with g++)
  bool setStdLib = 0;
  bool default_amdgpu_target = 1;
  bool compileOnly = 0;
  bool needCXXFLAGS = 0;  // need to add CXX flags to compile step
  bool needCFLAGS = 0;    // need to add C flags to compile step
  bool needLDFLAGS = 1;   // need to add LDFLAGS to compile step.
  bool fileTypeFlag = 0;  // to see if -x flag is mentioned
  bool hasOMPTargets = 0;  // If OMP targets is mentioned
  bool hasC = 0;          // options contain a c-style file
  // options contain a cpp-style file (NVCC must force recognition as GPU file)
  bool hasCXX = 0;
  // options contain a hip-style file (HIP-Clang must pass offloading options)
  bool hasHIP = 0;
  bool printHipVersion = 0;    // print HIP version
  bool printCXXFlags = 0;      // print HIPCXXFLAGS
  bool printLDFlags = 0;       // print HIPLDFLAGS
  bool runCmd = 1;
  bool buildDeps = 0;
  bool linkType = 1;
  bool setLinkType = 0;
  string hsacoVersion;
  bool funcSupp = 0;      // enable function support
  bool rdc = 0;           // whether -fgpu-rdc is on
  bool useHipfbCache = 0;

  string prevArg;  //  previous argument
  // TODO(hipcc): convert toolArgs to an array rather than a string
  string toolArgs;   // arguments to pass to the clang or nvcc tool
  string optArg;     // -O args
  vector<string> options, inputs;

  // TODO(hipcc): hipcc uses --amdgpu-target for historical reasons.
  // It should be replaced
  // by clang option --offload-arch.
  vector<string> targetOpts = {"--offload-arch=", "--amdgpu-target="};
  string targetsStr;
  // file followed by -o should not contibute in picking compiler flags
  bool skipOutputFile = false;

  const OsType& os = getOSInfo();
  string hip_compile_cxx_as_hip;
  if (var.hipCompileCxxAsHipEnv_.empty()) {
    hip_compile_cxx_as_hip = "1";
  } else {
    hip_compile_cxx_as_hip = var.hipCompileCxxAsHipEnv_;
  }

  HipGpuBinaryCache gpuBinaryCache(getHipVersion(), verbose & 0x8);

  string useHipfbCacheEnv = getEnvVariables().hipEnableHipfbCache;
  if (!useHipfbCacheEnv.empty() && std::stoi(useHipfbCacheEnv) != 0) {
    useHipfbCache = true;
  }

  string HIPLDARCHFLAGS;

  initializeHipCXXFlags();
  initializeHipCFlags();
  initializeHipLdFlags();
  string HIPCXXFLAGS, HIPCFLAGS, HIPLDFLAGS;
  HIPCFLAGS = getHipCFlags();
  HIPCXXFLAGS = getHipCXXFlags();
  HIPLDFLAGS = getHipLdFlags();
  string hipLibPath;
  string hipclangIncludePath , hipIncludePath, deviceLibPath;
  hipLibPath = getHipLibPath();
  const string& roccmPath = getRoccmPath();
  const string& hipPath = getHipPath();
  const PlatformInfo& platformInfo = getPlatformInfo();
  const string& rocclrHomePath = getRocclrHomePath();
  const string& hipClangPath = getCompilerPath();
  hipclangIncludePath = getCompilerIncludePath();
  hipIncludePath = getHipInclude();
  deviceLibPath = getDeviceLibPath();
  const string& hipVersion = getHipVersion();
  if (verbose & 0x2) {
    cout << "HIP_PATH=" << hipPath << endl;
    cout << "HIP_PLATFORM=" <<  PlatformTypeStr(platformInfo.platform) <<endl;
    cout << "HIP_COMPILER=" << CompilerTypeStr(platformInfo.compiler) <<endl;
    cout << "HIP_RUNTIME=" << RuntimeTypeStr(platformInfo.runtime) <<endl;
    cout << "ROCM_PATH=" << roccmPath << endl;
    cout << "HIP_ROCCLR_HOME="<< rocclrHomePath << endl;
    cout << "HIP_CLANG_PATH=" << hipClangPath <<endl;
    cout << "HIP_CLANG_INCLUDE_PATH="<< hipclangIncludePath <<endl;
    cout << "HIP_INCLUDE_PATH="<< hipIncludePath  <<endl;
    cout << "HIP_LIB_PATH="<< hipLibPath <<endl;
    cout << "DEVICE_LIB_PATH="<< deviceLibPath <<endl;
  }

  if (verbose & 0x4) {
    cout <<  "hipcc-args: ";
    for (unsigned int i = 1; i< argv.size(); i++) {
      cout <<  argv.at(i) << " ";
    }
    cout << endl;
  }


  for (unsigned int argcount = 1; argcount < argv.size(); argcount++) {
    // Save $arg, it can get changed in the loop.
    string arg = argv.at(argcount);
    // TODO(hipcc): figure out why this space removal is wanted.
    // TODO(hipcc): If someone has gone to the effort of
    // quoting the spaces to the shell
    // TODO(hipcc): why are we removing it here?
    regex toRemove("\\s+");
    // Remove whitespace
    string trimarg = hipBinUtilPtr_->replaceRegex(arg, toRemove, "");
    bool swallowArg = false;
    bool escapeArg = true;
    if (arg == "-c" || arg == "--genco" || arg == "-E") {
      compileOnly = true;
      needLDFLAGS  = false;
    }

    if (skipOutputFile) {
      // TODO(hipcc): handle filename with shell metacharacters
      toolArgs += " \"" + arg +"\"";
      prevArg = arg;
      skipOutputFile = 0;
      continue;
    }

    if (arg == "-o") {
      needLDFLAGS = 1;
      skipOutputFile = 1;
    }

    if ((trimarg == "-stdlib=libc++") && (setStdLib == 0)) {
      HIPCXXFLAGS += " -stdlib=libc++";
      setStdLib = 1;
    }

    // Check target selection option: --offload-arch= and --amdgpu-target=...
    for (unsigned int i = 0; i <targetOpts.size(); i++) {
      string targetOpt = targetOpts.at(i);
      // match arg with the starting of targetOpt
      string pattern = "^" + targetOpt + ".*";
      if (hipBinUtilPtr_->stringRegexMatch(arg, pattern))  {
        // If targets string is not empty,
        // add a comma before adding new target option value.
        targetsStr.size() >0 ? targetsStr += ",": targetsStr += "";
        targetsStr += arg.substr(targetOpt.size());  // argument of targetOpts
        default_amdgpu_target = 0;
        // Collect the GPU arch options and pass them to clang later.
        swallowArg = 1;
      }
    }  // end of for targetOpts for loop

    if (hipBinUtilPtr_->substringPresent(arg, "--genco")) {
      arg = "--cuda-device-only";
    }

    if (trimarg == "--version") {
      printHipVersion = 1;
    }
    if (trimarg == "--short-version") {
      printHipVersion = 1;
      runCmd = 0;
    }
    if (trimarg == "--cxxflags") {
      printCXXFlags = 1;
      runCmd = 0;
    }
    if (trimarg == "--ldflags") {
      printLDFlags = 1;
      runCmd = 0;
    }
    if (trimarg == "-M") {
      compileOnly = 1;
      buildDeps = 1;
    }
    if (trimarg == "-use_fast_math") {
      HIPCXXFLAGS += " -DHIP_FAST_MATH ";
      HIPCFLAGS += " -DHIP_FAST_MATH ";
    }
    if ((trimarg == "-use-staticlib") && (setLinkType == 0)) {
      linkType = 0;
      setLinkType = 1;
      swallowArg = 1;
    }
    if ((trimarg == "-use-sharedlib") && (setLinkType == 0)) {
      linkType = 1;
      setLinkType = 1;
    }
    if (trimarg == "--clean-cache") {
      gpuBinaryCache.cleanUpCache();
      return;
    }
    if (hipBinUtilPtr_->stringRegexMatch(arg, "^-O.*")) {
      optArg = arg;
    }
    if (hipBinUtilPtr_->substringPresent(
        arg, "--amdhsa-code-object-version=")) {
      arg = hipBinUtilPtr_->replaceStr(
            arg, "--amdhsa-code-object-version=", "");
      hsacoVersion = arg;
      swallowArg = 1;
    }

    // process linker response file for hip-clang
    // extract object files from static library and pass them directly to
    // hip-clang in command line.
    // TODO(hipcc): Remove this after hip-clang switch to lto and lld is able to
    // handle clang-offload-bundler bundles.
    if ((hipBinUtilPtr_->stringRegexMatch(arg, "^-Wl,@.*")) ||
       (hipBinUtilPtr_->stringRegexMatch(arg, "^@.*"))) {
      // arg will have options type(-Wl,@ or @) and filename
      vector<string> split_arg = hipBinUtilPtr_->splitStr(targetsStr, '@');
      string file = split_arg.at(1);
      ifstream in(file);
      if (!in.is_open()) {
        cout << "unable to open file for reading: " << file << endl;
        exit(-1);
      }
      string new_arg;
      string tmpdir = hipBinUtilPtr_->getTempDir();
      fs::path new_file = tmpdir;
      new_file /=  "response_file";
      ofstream out(new_file);
      if (!out.is_open()) {
        cout << "unable to open file for writing: " <<
                 new_file.string() << endl;
        exit(-1);
      }
      string line;
      while (getline(in, line)) {
        line = hipBinUtilPtr_->trim(line);
        if ((hipBinUtilPtr_->stringRegexMatch(line, ".*\\.a$")) ||
            (hipBinUtilPtr_->stringRegexMatch(line, ".*\\.lo$"))) {
          //## process static library for hip-clang
          //## extract object files from static library and
          //##  pass them directly to hip-clang.
          //## ToDo: Remove this after hip-clang switch to lto and
          //## lld is able to handle clang-offload-bundler bundles.
          string libFile  = line;
          string path = fs::absolute(line).string();
          // Check if all files in .a are object files.
          string cmd = "cd "+ tmpdir + "; ar xv " + path;
          SystemCmdOut sysOut;
          sysOut = hipBinUtilPtr_->exec(cmd.c_str());
          string cmdOut = sysOut.out;
          vector<string> objs = hipBinUtilPtr_->splitStr(cmdOut, '\n');
          bool allIsObj = 1;
          string realObjs = "";
          for (unsigned int i=0; i < objs.size(); i++) {
            string obj = objs.at(i);
            obj = hipBinUtilPtr_->trim(obj);
            regex toReplace("x - ");
            obj = hipBinUtilPtr_->replaceRegex(obj, toReplace, "");
            obj = "\"" + tmpdir + "/" + obj;
            cmd = "file " + obj;
            SystemCmdOut sysOut;
            sysOut = hipBinUtilPtr_->exec(cmd.c_str());
            string fileType = sysOut.out;
            bool isObj;
            (hipBinUtilPtr_->substringPresent(fileType, "ELF") ||
             hipBinUtilPtr_->substringPresent(fileType, "COFF")) ?
                                    isObj = true : isObj = false;
            allIsObj = allIsObj && isObj;
            if (isObj) {
              realObjs = realObjs + " " + obj;
            } else {
              inputs.push_back(obj);
              new_arg = "\"" + new_arg + obj + "\"";
            }
          }  // end of objs for loop
          realObjs = hipBinUtilPtr_->trim(realObjs);
          if (allIsObj) {
            out << line << "\n";
          } else if (!realObjs.empty()) {
            fs::path libFilefs = libFile;
            string libBaseName = libFilefs.stem().string();
            string libDir = libFilefs.parent_path().string();
            string libExt = libFilefs.extension().string();
            string  libBaseNameTemp = libBaseName + "XXXXXX";
            libBaseName = hipBinUtilPtr_->mktempFile(libBaseNameTemp) + libExt;
            cmd = "cd " + tmpdir + "; ar rc " + libBaseName + " " +realObjs;
            SystemCmdOut sysOut;
            sysOut = hipBinUtilPtr_->exec(cmd.c_str());
            string cmdOut = sysOut.out;
            out << tmpdir + "/"+ libBaseName + "\n";
          }
        } else if (hipBinUtilPtr_->stringRegexMatch(line, ".*\\.o$")) {
          string cmd = "file " + line;
          SystemCmdOut sysOut;
          sysOut = hipBinUtilPtr_->exec(cmd.c_str());
          string fileType = sysOut.out;
          bool isObj;
          (hipBinUtilPtr_->substringPresent(fileType, "ELF") ||
           hipBinUtilPtr_->substringPresent(fileType, "COFF")) ?
                                  isObj = true : isObj = false;
          if (isObj) {
            out << line << "\n";
          } else {
            inputs.push_back(line);
            new_arg = "\"" + new_arg + " " + line + "\"";
          }
        } else {
            out << line << "\n";
        }
      }  // end of while loop
        in.close();
        out.close();
        arg = "\"" + new_arg +" " +split_arg.at(0) + "\\" + new_file.string();
        escapeArg = 0;
      } else if ((hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.a$")) ||
                 (hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.lo$"))) {
        string new_arg = "";
        string tmpdir = hipBinUtilPtr_->getTempDir();
        string libFile = arg;
        string path = fs::absolute(arg).string();
        string cmd = "cd "+ tmpdir + "; ar xv " + path;
        SystemCmdOut sysOut;
        sysOut = hipBinUtilPtr_->exec(cmd.c_str());
        string cmdOut = sysOut.out;
        vector<string> objs = hipBinUtilPtr_->splitStr(cmdOut, '\n');
        bool allIsObj = 1;
        string realObjs = "";
        for (unsigned int i =0; i< objs.size(); i++) {
          string obj = objs.at(i);
          obj = hipBinUtilPtr_->trim(obj);
          regex toReplace("x - ");
          string replaceWith = "";
          obj = hipBinUtilPtr_->replaceRegex(obj, toReplace , replaceWith);
          obj = "\"" + tmpdir + "/" + obj + "\"";
          string cmd = "file " + obj;
          SystemCmdOut sysOut;
          sysOut = hipBinUtilPtr_->exec(cmd.c_str());
          string fileType = sysOut.out;
          bool isObj;
          isObj =  (hipBinUtilPtr_->substringPresent(fileType, "ELF") ||
                    hipBinUtilPtr_->substringPresent(fileType, "COFF"));
          if (hipBinUtilPtr_->substringPresent(fileType, "ELF")) {
            cmd = "readelf -e -W " + obj;
            SystemCmdOut sysOut;
            sysOut = hipBinUtilPtr_->exec(cmd.c_str());
            string sections = sysOut.out;
            isObj  = !(hipBinUtilPtr_->substringPresent(
                       sections, "__CLANG_OFFLOAD_BUNDLE__"));
          }
          allIsObj = (allIsObj && isObj);
          if (isObj) {
            realObjs = realObjs + " " + obj;
          } else {
            inputs.push_back(obj);
            if (new_arg != "") {
              new_arg += " ";
            }
            new_arg += "\"" + obj + "\"";
          }
        }  // end of objs for loop

        realObjs = hipBinUtilPtr_->trim(realObjs);
        if (allIsObj) {
          new_arg = arg;
        } else if (!realObjs.empty()) {
          fs::path libFilefs = libFile;
          string libBaseName = libFilefs.stem().string();
          string libDir = libFilefs.parent_path().string();
          string libExt = libFilefs.extension().string();
          string  libBaseNameTemp = libBaseName + "XXXXXX";
          libBaseName = hipBinUtilPtr_->mktempFile(
                        libBaseNameTemp) + libExt;
          string cmd = "cd " + tmpdir +"; ar rc " +
                       libBaseName + " " + realObjs;
          SystemCmdOut sysOut;
          sysOut = hipBinUtilPtr_->exec(cmd.c_str());
          string cmdOut = sysOut.out;
          new_arg += "\"" + tmpdir +"/" + libBaseName + "\"";
        }
        arg = "\"" + new_arg + "\"";
        escapeArg = 0;
        if (hipBinUtilPtr_->stringRegexMatch(toolArgs, ".*-Xlinker$")) {
          toolArgs = toolArgs.substr(0, -8);
          toolArgs = hipBinUtilPtr_->trim(toolArgs);
        }
    } else if (arg == "-x") {  // end of substring \.a || .lo section
        fileTypeFlag = 1;
    } else if ((arg == "c" && prevArg == "-x") || (arg == "-xc")) {
        fileTypeFlag = 1;
        hasC = 1;
        hasCXX = 0;
        hasHIP = 0;
    } else if ((arg == "c++" && prevArg == "-x") || (arg == "-xc++")) {
        fileTypeFlag = 1;
        hasC = 0;
        hasCXX = 1;
        hasHIP = 0;
    } else if ((arg == "hip" && prevArg == "-x") || (arg == "-xhip")) {
        fileTypeFlag = 1;
        hasC = 0;
        hasCXX = 0;
        hasHIP = 1;
    } else if (hipBinUtilPtr_->substringPresent(arg, "-fopenmp-targets=")) {
        hasOMPTargets = 1;
      // options start with -
    } else if (hipBinUtilPtr_->stringRegexMatch(arg, "^-.*")) {
        if  (arg == "-fgpu-rdc") {
          rdc = 1;
        } else if (arg == "-fno-gpu-rdc") {
          rdc = 0;
        }
        //# Process HIPCC options here:
        if (hipBinUtilPtr_->stringRegexMatch(arg, "^--hipcc.*")) {
          swallowArg = 1;
          // if $arg eq "--hipcc_profile") {  # Example argument here, hipcc
          //
          // }
          if (arg == "--hipcc-func-supp") {
            funcSupp = 1;
          } else if (arg == "--hipcc-no-func-supp") {
            funcSupp = 0;
          }
        } else {
          options.push_back(arg);
        }
      // print "O: <$arg>\n";
    } else if (prevArg != "-o") {
    // input files and libraries
    // Skip guessing if `-x {c|c++|hip}` is already specified.
    // Add proper file extension before each file type
    // File Extension                 -> Flag
    // .c                             -> -x c
    // .cpp/.cxx/.cc/.cu/.cuh/.hip    -> -x hip

    if (fileTypeFlag == 0) {
      if (hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.c$")) {
        hasC = 1;
        needCFLAGS = 1;
        toolArgs += " -x c";
      } else if ((hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.cpp$")) ||
                 (hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.cxx$")) ||
                 (hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.cc$")) ||
                 (hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.C$"))) {
        needCXXFLAGS = 1;
        if (hip_compile_cxx_as_hip == "0" || hasOMPTargets == 1) {
          hasCXX = 1;
        } else {
          hasHIP = 1;
          toolArgs += " -x hip";
        }
      } else if (((hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.cu$") ||
                   hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.cuh$")) &&
                   hip_compile_cxx_as_hip != "0") ||
                  (hipBinUtilPtr_->stringRegexMatch(arg, ".*\\.hip$"))) {
        needCXXFLAGS = 1;
        hasHIP = 1;
        toolArgs += " -x hip";
      }
    }
    if (hasC) {
      needCFLAGS = 1;
    } else if (hasCXX || hasHIP) {
      needCXXFLAGS = 1;
    }
    gpuBinaryCache.srcFileInfo.path = arg;
    if (useHipfbCache && hasHIP) {
      gpuBinaryCache.srcFileInfo.path = arg;
      gpuBinaryCache.enabled = true;
    }
    inputs.push_back(arg);
    // print "I: <$arg>\n";
    }
    // Produce a version of $arg where characters significant to the shell are
    // quoted. One could quote everything of course but don't bother for
    // common characters such as alphanumerics.
    // Do the quoting here because sometimes the $arg is changed in the loop
    // Important to have all of '-Xlinker' in the set of unquoted characters.
    // Windows needs different quoting, ignore for now
    if (os != windows && escapeArg) {
      regex reg("[^-a-zA-Z0-9_=+,.\/]");
      arg = regex_replace(arg, reg, "\\$&");
    }
    if (!swallowArg)
      toolArgs += " " + arg;
    prevArg = arg;
  }  // end of for loop

  // No AMDGPU target specified at commandline. So look for HCC_AMDGPU_TARGET
  if (default_amdgpu_target == 1) {
    if (!var.hccAmdGpuTargetEnv_.empty()) {
      targetsStr = var.hccAmdGpuTargetEnv_;
    } else if (os != windows) {
      // Else try using rocm_agent_enumerator
      string ROCM_AGENT_ENUM;
      ROCM_AGENT_ENUM = roccmPath + "/bin/rocm_agent_enumerator";
      targetsStr = ROCM_AGENT_ENUM +" -t GPU";
      SystemCmdOut sysOut = hipBinUtilPtr_->exec(targetsStr.c_str());
      regex toReplace("\n+");
      targetsStr = hipBinUtilPtr_->replaceRegex(sysOut.out, toReplace, ",");
    }
    default_amdgpu_target = 0;
  }
  // Parse the targets collected in targetStr
  // and set corresponding compiler options.
  vector<string> targets = hipBinUtilPtr_->splitStr(targetsStr, ',');
  string GPU_ARCH_OPT = " --offload-arch=";

  for (auto &val : targets) {
    // Ignore 'gfx000' target reported by rocm_agent_enumerator.
    if (val != "gfx000") {
      vector<string> procAndFeatures = hipBinUtilPtr_->splitStr(val, ':');
      size_t len = procAndFeatures.size();
      // proc and features
      assertm(procAndFeatures.size() >= 1, "Pass the correct device/feature");
      for (size_t i = 1; i < len; i++) {
          // fixme: currently it checks only for validity of the feature string.
          // does not check if the device supports the feature or not
          // e.g. vega10 does not support sramecc
          if (knownFeatures.find(procAndFeatures.at(i)) == knownFeatures.end()) {
            cout <<  "Warning: The Feature: "<< procAndFeatures.at(i) <<
                     " is unknown. Correct compilation is not guaranteed.\n";
          }
      }
      string GPU_ARCH_ARG;
      GPU_ARCH_ARG = GPU_ARCH_OPT + val;

      HIPLDARCHFLAGS += GPU_ARCH_ARG;
      if (hasHIP) {
        HIPCXXFLAGS += GPU_ARCH_ARG;
      }
/* <<<<<<< HEAD
======= */
      for (auto it = knownTargets.cbegin(); it != knownTargets.cend(); it++) {
        if (procName == *it) break;
        else if (it + 1 == knownTargets.cend())
          cout << "Warning: The specified HIP target: "<< val <<  " is unknown. Correct compilation is not guaranteed.\n";
      }
// >>>>>>> code cache
    }  // end of val != "gfx000"
  }  // end of targets for loop

  if (targets.size() > 1) {
    gpuBinaryCache.multTargets = true;
  }

  string HCC_EXTRA_LIBRARIES;
  if (hsacoVersion.size() > 0) {
    if (compileOnly == 0) {
      HIPLDFLAGS += " -mcode-object-version=" + hsacoVersion;
    } else {
      HIPCXXFLAGS += " -mcode-object-version=" + hsacoVersion;
    }
  }

  // rocm_agent_enumerator failed! Throw an error and die if linking is required
  if (default_amdgpu_target == 1 && compileOnly == 0) {
    // TODO(agunashe) exit from function
    cout <<  "No valid AMD GPU target was either specified or found."
        << "Please specify a valid target using --offload-arch=<target>.\n";
  }
  HCC_EXTRA_LIBRARIES ="\n";  // TODO(agunashe) write to env

  if (buildDeps) {
    HIPCXXFLAGS += " --cuda-host-only";
  }
  // Add --hip-link only if it is compile only and -fgpu-rdc is on.
  if (rdc && !compileOnly) {
    HIPLDFLAGS += " --hip-link";
    HIPLDFLAGS += HIPLDARCHFLAGS;
  }

  // hipcc currrently requires separate compilation of source files,
  // ie it is not possible to pass
  // CPP files combined with .O files
  // Reason is that NVCC uses the file extension to determine
  // whether to compile in CUDA mode or
  // pass-through CPP mode.
  // Set default optimization level to -O3 for hip-clang.
  if (optArg.empty()) {
    HIPCXXFLAGS += " -O3";
    HIPCFLAGS += " -O3";
    HIPLDFLAGS += " -O3";
  }

  if (!funcSupp && optArg != "-O0" && hasHIP) {
    HIPCXXFLAGS +=
    " -mllvm -amdgpu-early-inline-all=true -mllvm -amdgpu-function-calls=false";
    if (needLDFLAGS && !needCXXFLAGS) {
      HIPLDFLAGS +=
      " -mllvm -amdgpu-early-inline-all=true"
      " -mllvm -amdgpu-function-calls=false";
    }
  }

  if (hasHIP) {
    fs::path bitcodeFs = roccmPath;
    bitcodeFs /= "amdgcn/bitcode";
    if (deviceLibPath != bitcodeFs.string()) {
      string hip_device_lib_str = " --hip-device-lib-path=\""
                                  + deviceLibPath + "\"";
      HIPCXXFLAGS += hip_device_lib_str;
    }
  }
  if (os != windows) {
    HIPLDFLAGS += " -lgcc_s -lgcc -lpthread -lm -lrt";
  }

  if (os != windows && !compileOnly) {
    string hipClangVersion, toolArgTemp;
    if (linkType == 0) {
      toolArgTemp = " -L"+ hipLibPath + "-lamdhip64 -L" +
                      roccmPath+ "/lib -lhsa-runtime64 -ldl -lnuma " + toolArgs;
      toolArgs = toolArgTemp;
    } else {
      toolArgTemp =  toolArgs + " -Wl,--enable-new-dtags -Wl,-rpath=" + hipLibPath + ":"
                    + roccmPath+"/lib -lamdhip64 ";
      toolArgs =  toolArgTemp;
    }

    hipClangVersion = getCompilerVersion();
    // To support __fp16 and _Float16, explicitly link with compiler-rt
    toolArgs += " -L" + hipClangPath + "/../lib/clang/" +
                hipClangVersion + "/lib/linux -lclang_rt.builtins-x86_64 ";
  }
  if (!var.hipccCompileFlagsAppendEnv_.empty()) {
    HIPCXXFLAGS += " " + var.hipccCompileFlagsAppendEnv_ + " ";
    HIPCFLAGS += " " + var.hipccCompileFlagsAppendEnv_ + " ";
  }
  if (!var.hipccLinkFlagsAppendEnv_.empty()) {
    HIPLDFLAGS += " " + var.hipccLinkFlagsAppendEnv_ + " ";
  }
  // TODO(hipcc): convert CMD to an array rather than a string
  string compiler;
  compiler = getHipCC();
  string CMD = compiler;
  string ALL_ARGS = "";
  if (needCFLAGS) {
    ALL_ARGS += " " + HIPCFLAGS;
  }

  if (needCXXFLAGS) {
    ALL_ARGS += " " + HIPCXXFLAGS;
  }

  if (needLDFLAGS && !compileOnly) {
    ALL_ARGS += " " + HIPLDFLAGS;
  }

  ALL_ARGS += " " + toolArgs;
  gpuBinaryCache.clangPath = compiler;
  gpuBinaryCache.clangArgs = ALL_ARGS;
  CMD += ALL_ARGS;

  if (verbose & 0x1) {
    cout << "hipcc-cmd: " <<  CMD << "\n";
  }

  if (printHipVersion) {
    if (runCmd) {
      cout <<  "HIP version: ";
    }
    cout << hipVersion << endl;
  }
  if (printCXXFlags) {
    cout << HIPCXXFLAGS;
  }
  if (printLDFlags) {
    cout << HIPLDFLAGS;
  }

  if (runCmd) {
    SystemCmdOut sysOut;
    bool useCachedHipfb = false;
    bool updatedCache = false;

    // try using cached hipfb binary.
    if (gpuBinaryCache.enabled) {
      useCachedHipfb = gpuBinaryCache.findCachedHipfb(CMD);
      // If required hipfb is not found in the cache, find next available cache entry
      // to save the new generated hipfb.
      if (useCachedHipfb) {
        gpuBinaryCache.addUseHipfbOption(CMD);
      }
      else {
        if (gpuBinaryCache.findAvailableEntry()) {
          if (gpuBinaryCache.preprocessSourceToFile(CMD)) {
            gpuBinaryCache.addSaveHipfbOption(CMD);
            updatedCache = true;
          }
        }
      }
    }

    //std::cout << "Started build for " << gpuBinaryCache.srcFileInfo.path << std::endl;

    sysOut = hipBinUtilPtr_->exec(CMD.c_str(), true);
    string cmdOut = sysOut.out;
    int CMD_EXIT_CODE = sysOut.exitCode;
/** <<<<<<< HEAD
    if (CMD_EXIT_CODE !=0) {
      cout <<  "failed to execute:"  << CMD << std::endl;
======= */

    //std::cout << "Finished build for " << gpuBinaryCache.srcFileInfo.path << std::endl;

    if (updatedCache) {
      gpuBinaryCache.updateFilesPostBuild(CMD_EXIT_CODE != 0);
    }

    if (CMD_EXIT_CODE == -1) {
      cout <<  "failed to execute: $!\n";
    } else if (CMD_EXIT_CODE & 127) {
      string childOut;
      int childCode;
      (CMD_EXIT_CODE & 127),  (CMD_EXIT_CODE & 128) ?
              childOut = "with" : childOut = "without";
      (CMD_EXIT_CODE & 127),  (CMD_EXIT_CODE & 128) ?
              childCode = (CMD_EXIT_CODE & 127) :
              childCode = (CMD_EXIT_CODE & 128);
      cout <<  "child died with signal " << childCode << "," << childOut <<
                          " coredump "<< " for cmd: " << CMD << endl;
    } else {
      CMD_EXIT_CODE = CMD_EXIT_CODE >> 8;
// >>>>>>> code cache
    }
    exit(CMD_EXIT_CODE);
  }  // end of runCmd section
}   // end of function

HipGpuBinaryCache::HipGpuBinaryCache(const std::string& hipVer, bool verb)
  : hipVersion(hipVer), verbose(verb)
{
    const char* homedir;
    if ((homedir = getenv("HOME")) != NULL) {
      cacheDir = std::string(homedir) + "/" + CACHE_DIR_SUFFIX;
    }
}

bool HipGpuBinaryCache::findCachedHipfb(std::string& cmd)
{
    //std::cout << "Entering findCachedHipfb() for " << srcFileInfo.path << std::endl;
    auto pos = srcFileInfo.path.rfind('/');
    srcFileInfo.name = srcFileInfo.path.substr(pos == std::string::npos ? 0 : pos + 1);

    // The cache data file is named "<N>.json", where
    // N = (size of cource file & DATA_FILES_MASK).
    struct stat stat_buf;
    int rc = stat(srcFileInfo.path.c_str(), &stat_buf);
    if (rc == -1)
      return false;
    else {
      srcFileInfo.size = stat_buf.st_size;
      srcFileInfo.modTime = stat_buf.st_mtim.tv_sec;
    }
/*
    if (srcFileInfo.size > SRC_SIZE_LIMIT) {
      if (verbose) {
        std::cout << "[HIPFB CACHE] ... Not using cache for " << srcFileInfo.name <<
                     " : Source file size limit exceeded." << std::endl;
      }
      return false;
    }
*/
    int dataFileNum = srcFileInfo.size & DATA_FILES_MASK;

    // Open, lock and parse the cache data file if it exists.
    if (cacheDir.empty()) {
      //std::cout << "Leaving findCachedHipfb() [cache dir empty] for " << srcFileInfo.name << std::endl;
      return false;
    }

    std::string dataFilePath = cacheDir + "/" + std::to_string(dataFileNum) + ".json";

    //std::cout << "findCachedHipfb(): Trying SH lock for " << srcFileInfo.name << std::endl;
    FileLock fl(dataFilePath.c_str());
    //std::cout << "findCachedHipfb(): Acquired SH lock for " << srcFileInfo.name << std::endl;

    std::ifstream dataFile(dataFilePath);
    if (!dataFile.is_open()) {
      //std::cout << "findCachedHipfb(): Released SH lock for " << srcFileInfo.name << std::endl;
      //std::cout << "Leaving findCachedHipfb() [no data file] for " << srcFileInfo.name << std::endl;
      return false;
    }

    // std::cout << "Reading data file in findCachedHipfb : " << dataFilePath << 
    //              " for src file " << srcFileInfo.name << std::endl;
    nlohmann::json jdata;
    dataFile >> jdata;
    dataFile.close();
    fl.release();
    //std::cout << std::setw(4) << jdata << std::endl;
    //std::cout << "findCachedHipfb(): Released SH lock for " << srcFileInfo.name << std::endl;

    // Try to find entries that could match the source file.
    // 1. Perform the "fast" check: compare the source file name, size & modification time
    //    and the rocm (HIP) version.
    std::set<nlohmann::json> matchingEntries;
    for (auto &j : jdata) {
      if (/*j.contains("source_size") &&*/ srcFileInfo.size == j["source_size"] &&
          /*j.contains("source_mod_time") && srcFileInfo.modTime == j["source_mod_time"] && */
          /*j.contains("source_name") &&*/ srcFileInfo.name == j["source_name"] &&
          /*j.contains("hip_version") &&*/ hipVersion == j["hip_version"]) {
        matchingEntries.insert(j);
      }
    }

    if (matchingEntries.empty()) {
      //std::cout << "Leaving findCachedHipfb() [no matching entry] for " << srcFileInfo.name << std::endl;
      return false;
    }

    // 2. Check the clang arguments.
    for (auto it = matchingEntries.begin(); it != matchingEntries.end(); ) {
      if (clangArgs != (*it)["clang_options"])
        it = matchingEntries.erase(it);
      else
        it++;
    }

    if (matchingEntries.empty()) {
      //std::cout << "Leaving findCachedHipfb() [no matching entry] for " << srcFileInfo.name << std::endl;
      return false;
    }

    // 3. Now perform the "heaviest" check of the remaining potentially matching
    //    cache entries: compare the size and content of preprocessed source files.
    nlohmann::json matchingEntry = nullptr;

    for (auto &jentry : matchingEntries) {
      std::stringstream prepText, cachedPrepText;
      std::string s1, s2;
      bool match = true;

      if (!preprocessSource(srcFileInfo.name, prepText))
        continue;
      if (!readCachedPrepSource(jentry, cachedPrepText))
        continue;

      prepText.seekg(0, std::ios::end);
      size_t prepSize = prepText.tellg();
      cachedPrepText.seekg(0, std::ios::end);
      size_t cachedSize = cachedPrepText.tellg();
      if (prepSize != cachedSize)
        continue;
      prepText.seekg(0, std::ios::beg);
      cachedPrepText.seekg(0, std::ios::beg);
      
      while (std::getline(prepText, s1) && std::getline(cachedPrepText, s2)) {
        if (s1 != s2) {
          match = false;
          break;
        }
      }

      if (match){
        matchingEntry = jentry;
        break;
      }
    }

    if (matchingEntry.empty()) {
      //std::cout << "Leaving findCachedHipfb() [no matching entry] for " << srcFileInfo.name << std::endl;
      return false;
    }

    // Found a matching cache entry -- modify the clang command line
    // to make clang use cached hipfb instead of building the Device part of the source.
    cacheEntryInfo.cachedHipfbName = matchingEntry["cached_hipfb"];
    cacheEntryInfo.subDir = matchingEntry["cache_subdir"];
    cacheEntryInfo.howAdded = ENTRY_INFO::FOUND_CACHED;

    //std::cout << "Leaving findCachedHipfb() [found cached entry] for " << srcFileInfo.name << std::endl;

    return true;
}

bool HipGpuBinaryCache::preprocessSource(const std::string& srcFileName, std::stringstream& prepText)
{
  std::string tmpDir = HipBinUtil::getTempDir();
  std::string cuiFilePath = tmpDir + "/" + srcFileName + ".cui";

  std::string clangCmd = clangPath;

  // Preprocessor does not accept multiple targets, so remove all targets except the 1st one.
  if (multTargets) {
    std::string singleTargetArgs, arg;
    std::istringstream iss(clangArgs);
    bool addedTarget = false;
    while (std::getline(iss, arg, ' ')) {
      if (arg.find("--offload-arch") != std::string::npos) {
        if (!addedTarget) {
          singleTargetArgs += (arg + " ");
          addedTarget = true;
        }
      }
      else {
          singleTargetArgs += (arg + " ");
      }
    }
    clangCmd += (" " + singleTargetArgs);
  } else {
    clangCmd += (" " + clangArgs);
  }

  clangCmd += (" -E --cuda-device-only -o " + cuiFilePath);

  auto status = HipBinUtil::exec(clangCmd.c_str());
  if (status.exitCode != 0) return false;

  // Check if size of the generated .cui file exceeds the limit.
  struct stat stat_buf;
  int rc = stat(cuiFilePath.c_str(), &stat_buf);
  if (rc == -1) return false;

/*  
  if (stat_buf.st_size > CUI_SIZE_LIMIT) {
    if (verbose) {
      std::cout << "[HIPFB CACHE] ... Not using cache for " << srcFileInfo.name <<
                   " : cui file size limit exceeded." << std::endl;
    }
    return false;
  }
*/

  std::ifstream cuiFile(cuiFilePath);
  if (cuiFile.is_open()) {
    prepText << cuiFile.rdbuf();
    fs::remove(cuiFilePath);
    return true;
  }

  return false;
}

bool HipGpuBinaryCache::preprocessSourceToFile(const std::string& cmd)
{
    //std::cout << "Started preprocessing for " << srcFileInfo.name << std::endl;

    std::string cuiFilePath = cacheDir + "/" + cacheEntryInfo.subDir + "/" + srcFileInfo.name + ".cui";

    std::string prepCmd, arg;
    std::istringstream iss(cmd);

    // Preprocessor does not accept multiple targets, so remove all targets except the 1st one.
    if (!multTargets) {
      while (std::getline(iss, arg, ' ')) {
        prepCmd += (arg + " ");
        if (arg == "-o") {
          prepCmd += (cuiFilePath + " -E --cuda-device-only ");
          std::getline(iss, arg, ' ');
        }
      }
    }
    else {
      bool addedTarget = false;
      while (std::getline(iss, arg, ' ')) {
        if (arg == "-o") {
          // Replace the output file path in the clang command line.
          prepCmd += (arg + " " + cuiFilePath + " -E --cuda-device-only ");
          std::getline(iss, arg, ' ');
        }
        else if (arg.find("--offload-arch") != std::string::npos) {
          if (!addedTarget) {
            // Add only one target.
            prepCmd += (arg + " ");
            addedTarget = true;
          }
        }
        else {
          prepCmd += (arg + " ");
        }
      }
    }

    auto status = HipBinUtil::exec(prepCmd.c_str());
    if (status.exitCode != 0) return false;

    //std::cout << "Finished preprocessing for " << srcFileInfo.name << std::endl;

    return true;
}

bool HipGpuBinaryCache::readCachedPrepSource(const nlohmann::json& jentry, std::stringstream& text)
{
  std::string cacheSubdir = jentry["cache_subdir"];
  std::string cachedCuiPath = cacheDir + "/" + cacheSubdir + "/";
  cachedCuiPath += jentry["cached_cui"];
  std::ifstream cachedCuiFile(cachedCuiPath);
  if (cachedCuiFile.is_open()) {
    text << cachedCuiFile.rdbuf();
    return true;
  }

  return false;
}

bool HipGpuBinaryCache::findAvailableEntry()
{
  // If cache directory or cache description file does not exist, create them.
  struct stat info;
  std::string cacheFilePath = cacheDir + "/" + CACHE_DESC_FILE_NAME;
  std::ifstream cacheFile(cacheFilePath);

  //std::cout << "Entering findAvailableEntry() for " << srcFileInfo.name << std::endl;

  if(stat(cacheDir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR) ||
   !cacheFile.is_open()) {
    if (initCache()) {
      cacheFile.open(cacheFilePath);
    }
    else {
      std::cout << "[HIPFB CACHE] !!! Failed to initialize HIPFB cache." << std::endl;
      return false;
    }
  }

  // Lock, read and parse the cache description file
  //std::cout << "findAvailableEntry(): Trying EX lock for " << srcFileInfo.name << std::endl;
  FileLock fl(cacheFilePath, true);
  //std::cout << "findAvailableEntry(): Acquired EX lock for " << srcFileInfo.name << std::endl;
  //std::cout << "Reading cache desc file for " << srcFileInfo.name << std::endl;
  cacheFile >> cacheDesc;
  std::cout << std::setw(4) << cacheDesc << std::endl;
  cacheFile.close();
  //std::cout << "findAvailableEntry(): Read cache desc for " << srcFileInfo.name << std::endl;

  size_t cacheTotalSize = cacheDesc["total_size"];
  int topSubDirNum = cacheDesc["top_subdir"];
  bool found = false;

  // If there are available (empty) sub-folders and the total cache size
  // does not exceed the limit, use the first available sub-folder.
  if (cacheTotalSize + ENTRY_EST_SIZE < CACHE_SIZE_LIMIT) {
    // First, try to find an empty sub-directory.
    nlohmann::json &emptySlots = cacheDesc["empty_subdirs"];
    nlohmann::json pendingSubDirs = cacheDesc["pending_subdirs"];
    if (!emptySlots.empty()) {
      // Get the 1st available empty slot which is not in the list of pending slots.
      size_t emptySlotsNum = emptySlots.size();
      for (int i = 0; i < emptySlotsNum; i++) {
        if (!pendingSubDirs.contains(emptySlots[i])) {
          cacheEntryInfo.subDirNum = emptySlots[i];
          cacheEntryInfo.subDir = getSubDirName(cacheEntryInfo.subDirNum);
          emptySlots.erase(i);
          cacheEntryInfo.howAdded = ENTRY_INFO::USED_EMPTY_SUBDIR;
          found = true;
          //std::cout << "findAvailableEntry() [using empty subdir] for " << srcFileInfo.name << std::endl;
          break;
        }
      }
    }
    if (!found && topSubDirNum < MAX_SUBDIR_NUM) {
      // If there are sub-directories that have not been used yet, use the 1st available.
      int dirNum = topSubDirNum;
      std::vector<int> pendingDirNums = pendingSubDirs;      
      while (++dirNum <= MAX_SUBDIR_NUM) {
        bool dirIsInPendingList = false;
        for (int pndDir : pendingDirNums) {
          if (pndDir == dirNum) {
            dirIsInPendingList = true;
            break;
          }
        }
        if (!dirIsInPendingList) {
          cacheEntryInfo.subDirNum = dirNum;
          cacheEntryInfo.subDir = getSubDirName(cacheEntryInfo.subDirNum);
          found = createCacheSubDir();
          if (found) {
            cacheEntryInfo.howAdded = ENTRY_INFO::CREATED_NEW_SUBDIR;
            //std::cout << "[Created new subdir]: " << dirNum <<  " for " << srcFileInfo.name << std::endl;
            break; // Exit from "while (dirNum <= MAX_SUBDIR_NUM)"
          }
        }
      }
    }
  }

  if (!found) {
    // There are no available subfolders or the max cache size is exceeded.
    // Evict the oldest cache entry (or multiple entries if necessary).
    cacheEntryInfo.subDirNum = evictOldEntries();
    if (cacheEntryInfo.subDirNum != -1) {
        cacheEntryInfo.subDir = getSubDirName(cacheEntryInfo.subDirNum);
        cacheEntryInfo.howAdded = ENTRY_INFO::EVICTED_OLD_ENTRY;
        found = true;
    }
  }

  if (found) {
    // Add found sub-dir to the list of "pending" ones so that other hipcc processes
    // do not try to use it. Write updated data to the cache desc file.
    if (cacheEntryInfo.howAdded != ENTRY_INFO::EVICTED_OLD_ENTRY) {
      cacheDesc["pending_subdirs"].push_back(cacheEntryInfo.subDirNum);
    }
    updateCacheDescFile();
    //std::cout << "findAvailableEntry() [added pending subdir] for " << srcFileInfo.name << std::endl;
  }

  //std::cout << "findAvailableEntry(): Released EX lock for " << srcFileInfo.name << std::endl;
  //std::cout << "Leaving findAvailableEntry() for " << srcFileInfo.name << std::endl;
  return found;
}

bool HipGpuBinaryCache::createCacheSubDir()
{
  if (cacheEntryInfo.subDir.empty()) {
    cacheEntryInfo.subDir = getSubDirName(cacheEntryInfo.subDirNum);
  }

  // Create a cache sub-directory if it does not exist.
  struct stat info;
  std::string subDirPath = cacheDir + "/" + cacheEntryInfo.subDir;
  if(stat(subDirPath.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
    // TODO: Move creating a directory to the hipBin_util.h.
    int ret = mkdir(subDirPath.c_str(), S_IRWXU);
    return (ret == 0);
  }

  return true;
}

void HipGpuBinaryCache::addSaveHipfbOption(std::string& cmd)
{
  std::string hipfbFilePath = cacheDir + "/" + cacheEntryInfo.subDir + "/" + srcFileInfo.name + ".hipfb";
  cmd += (" --save-hipfb=" + hipfbFilePath);
}

bool HipGpuBinaryCache::updateFilesPostBuild(bool buildFailed)
{
  bool result = true;
  uint32_t entryID = 0;
  size_t hipfbFileSize, cuiFileSize;
  std::string cacheSubDirPath = cacheDir + "/" + cacheEntryInfo.subDir + "/";
  std::string hipfbFilePath = cacheSubDirPath + srcFileInfo.name + ".hipfb";
  std::string cuiFilePath = cacheSubDirPath + srcFileInfo.name + ".cui";
  struct stat stat_buf;

  if (stat(hipfbFilePath.c_str(), &stat_buf) == -1)
    return false;
  else
    hipfbFileSize = stat_buf.st_size;
  if (stat(cuiFilePath.c_str(), &stat_buf) == -1)
    return false;
  else
    cuiFileSize = stat_buf.st_size;

  if (!buildFailed) {
    result = updateDataFilesPostBuild(cuiFileSize + hipfbFileSize, entryID);
  }

  if (result) {
    result = updateCacheDescFilePostBuild(cuiFileSize + hipfbFileSize, entryID, buildFailed);
  }

  return result;
}

bool HipGpuBinaryCache::updateDataFilesPostBuild(size_t subdirSize, uint32_t& entryID)
{
  int dataFileNum = srcFileInfo.size & DATA_FILES_MASK;
  std::string dataFileName = std::to_string(dataFileNum) + ".json";
  std::string dataFilePath = cacheDir + "/" + dataFileName;

  //std::cout << "Entering updateDataFilesPostBuild() for " << srcFileInfo.name << std::endl;

  // Lock and read the cache description file
  //std::cout << "updateDataFilesPostBuild(): Trying EX lock for " << srcFileInfo.name << std::endl;
  FileLock fl(cacheDir + "/" + CACHE_DESC_FILE_NAME, true);
  //std::cout << "updateDataFilesPostBuild(): Acquired EX lock for " << srcFileInfo.name << std::endl;
  std::string cacheFilePath = cacheDir + "/" + CACHE_DESC_FILE_NAME;
  std::ifstream cacheFile(cacheFilePath);
  if (cacheFile.is_open()) {
    //std::cout << "updateDataFilesPostBuild() [Reading cache desc file] for " << srcFileInfo.name << std::endl;
    cacheFile >> cacheDesc;
    cacheFile.close();
    //std::cout << std::setw(4) << cacheDesc << std::endl;
    //std::cout << "updateDataFilesPostBuild() [Read cache desc file] for " << srcFileInfo.name << std::endl;
  }

  std::ifstream dataFile(dataFilePath);
  nlohmann::json entries;
  if (dataFile.is_open()) {
    // std::cout << "updateDataFilesPostBuild() Reading data file: " << dataFileNum << 
    //              " for " << srcFileInfo.name << std::endl;
    dataFile >> entries;
    dataFile.close();
    //std::cout << std::setw(4) << entries << std::endl;
  }

  //std::cout << "updateDataFilesPostBuild() [Read data file] for " << srcFileInfo.name << std::endl;

  // Open the data file containing the old "newest entry".
  // Link new entry to the previous "newest" entry.
  nlohmann::json oldNewest = cacheDesc["newest_entry"];
  uint32_t oldNewestDataFileNum = oldNewest.empty() ? 0 : (uint32_t) oldNewest[0];
  uint32_t oldNewestEntryNum = oldNewest.empty() ? 0 : (uint32_t) oldNewest[1];
  entryID = entries.size();

  if (!oldNewest.empty()) {
    if (oldNewestDataFileNum == dataFileNum) {
      nlohmann::json &oldNewestEntry = entries[oldNewestEntryNum];
      oldNewestEntry["next_entry"] = { dataFileNum, entryID };
    }
    else {
      nlohmann::json oldData;
      std::string oldDataFilePath = cacheDir + "/" + std::to_string(oldNewestDataFileNum) + ".json";
      std::ifstream oldDataFile(oldDataFilePath);
      if (oldDataFile.is_open()) {
        // std::cout << "updateDataFilesPostBuild() Reading old data file: " << oldNewestDataFileNum << 
        //              " for " << srcFileInfo.name << std::endl;
        oldDataFile >> oldData;
        oldDataFile.close();
        //std::cout << std::setw(4) << oldData << std::endl;
      }

      nlohmann::json &oldNewestEntry = oldData[oldNewestEntryNum];
      oldNewestEntry["next_entry"] = { dataFileNum, entryID };

      std::ofstream oldDataFileW(oldDataFilePath, std::ios::trunc);
      if (oldDataFileW.is_open()) {
        // std::cout << "updateDataFilesPostBuild() Updating old data file: " << oldNewestDataFileNum << 
        //              " for " << srcFileInfo.name << std::endl;
        oldDataFileW << std::setw(4) << oldData << std::endl;
        oldDataFileW.close();
        //std::cout << std::setw(4) << oldData << std::endl;
      }
    }
  }

  //std::cout << "updateDataFilesPostBuild() [Updated old data file] for " << srcFileInfo.name << std::endl;

  // Add new entry descriptor to the corresponding cache data file.
  nlohmann::json jentry;

  jentry["_ID_"]            = entries.size();
  jentry["source_name"]     = srcFileInfo.name;
  jentry["source_mod_time"] = srcFileInfo.modTime;
  jentry["source_size"]     = srcFileInfo.size;
  jentry["clang_options"]   = clangArgs;
  jentry["hip_version"]     = hipVersion;
  jentry["cache_subdir"]    = cacheEntryInfo.subDir;
  jentry["cached_cui"]      = srcFileInfo.name + ".cui";
  jentry["cached_hipfb"]    = srcFileInfo.name + ".hipfb";
  jentry["next_entry"]      = nlohmann::json::array();
  jentry["subdir_size"]     = subdirSize;

  // Add new entry to the data file.
  entries.push_back(jentry);

  // Write the data file.
  std::ofstream dataFileW(dataFilePath, std::ios::trunc);
  if (dataFileW.is_open()) {
      // std::cout << "updateDataFilesPostBuild() Updating data file: " << dataFileNum << 
      //              " for " << srcFileInfo.name << std::endl;
    dataFileW << std::setw(4) << entries << std::endl;
    dataFileW.close();
    //std::cout << std::setw(4) << entries << std::endl;
  }

  return true;
}

bool HipGpuBinaryCache::updateCacheDescFilePostBuild(size_t subdirSize, uint32_t entryID, bool buildFailed)
{
  // Update the cache desc file.
  size_t totalSize = cacheDesc["total_size"];
  totalSize += subdirSize;
  cacheDesc["total_size"] = totalSize;

  // Remove new entry from the "pending" list and update the cache description
  // according to the way the entry was added.
  nlohmann::json& pendingSubdirs = cacheDesc["pending_subdirs"];
  for (int i = 0; i < pendingSubdirs.size(); i++) {
    if (pendingSubdirs[i] == cacheEntryInfo.subDirNum) {
      pendingSubdirs.erase(i);
      break;
    }
  }

  // If the compilation has failed, just delete all possibly remaining files
  // and add the current cache sub-directory to the list of empty entries.
  if (buildFailed) {
    const std::string cacheSubDirPath = cacheDir + "/" + cacheEntryInfo.subDir;
    for (const auto& entry : fs::directory_iterator(cacheSubDirPath)) {
      fs::remove(entry.path());
    }

    nlohmann::json &emptySubdirs = cacheDesc["empty_subdirs"];
    emptySubdirs.push_back(cacheEntryInfo.subDirNum);
    updateCacheDescFile();

    return true;
  }

  int dataFileNum = srcFileInfo.size & DATA_FILES_MASK;
  if (cacheDesc["newest_entry"].empty()) {
    cacheDesc["oldest_entry"] = { dataFileNum, entryID };
  }
  cacheDesc["newest_entry"] = { dataFileNum, entryID };

  //std::cout << std::setw(4) << cacheDesc << std::endl;

  if (cacheEntryInfo.howAdded == ENTRY_INFO::CREATED_NEW_SUBDIR) {
    int topSubdir = cacheDesc["top_subdir"];
    if (cacheEntryInfo.subDirNum > topSubdir) {
      cacheDesc["top_subdir"] = cacheEntryInfo.subDirNum;
    }
  }
  else if (cacheEntryInfo.howAdded == ENTRY_INFO::USED_EMPTY_SUBDIR) {
    nlohmann::json& emptySubdirs = cacheDesc["empty_subdirs"];
    for (int i = 0; i < emptySubdirs.size(); i++) {
      if (emptySubdirs[i] == cacheEntryInfo.subDirNum) {
        emptySubdirs.erase(i);
        break;
      }
    }
  }

  // Re-write the cache description file.
  updateCacheDescFile();

  if (verbose) {
    std::cout << "[HIPFB CACHE] >>> Cached hipfb for " << srcFileInfo.name <<
                 " to cache subfolder: " << cacheEntryInfo.subDir << std::endl;
  }

  //std::cout << "updateDataFilesPostBuild() [Updated cache desc file] for " << srcFileInfo.name << std::endl;
  //std::cout << "updateDataFilesPostBuild(): Released EX lock for " << srcFileInfo.name << std::endl;

  return true;
}

std::string HipGpuBinaryCache::getSubDirName(int subDirNum)
{
  std::string dirName;
  // Add leading zeros to the subdir name.
  int m = (MAX_SUBDIR_NUM < 100 ? 100 : (MAX_SUBDIR_NUM < 1000 ? 1000 : 
           (MAX_SUBDIR_NUM < 10000 ? 10000 : (MAX_SUBDIR_NUM < 100000 ? 10000 : 100000))));
  while ((m /= 10) > subDirNum) dirName += "0";
  if (subDirNum > 0) dirName += std::to_string(subDirNum);  

  return dirName;
}

int HipGpuBinaryCache::evictOldEntries()
{
  // In the cache description file, the entries are listed in the date of creation order.
  // So, first entries are the oldest.
  int firstEvicted = -1;
  int numEvictedEntries = 0;

  size_t totalSize = cacheDesc["total_size"];
  nlohmann::json &emptySubdirs = cacheDesc["empty_subdirs"];
  nlohmann::json &oldestEntry = cacheDesc["oldest_entry"];

  if (oldestEntry.size() < 2) return -1;

  std::pair<uint32_t, uint32_t> entry = {oldestEntry[0], oldestEntry[1]};
  bool stop = false;

  for (int i=0; !stop; i++) {
    int evictedDir = evictEntry(entry, totalSize);
    oldestEntry = {std::get<0>(entry), std::get<1>(entry)};
    if (i == 0) {
      firstEvicted = evictedDir;
    }
    else {
      emptySubdirs.push_back(evictedDir);
    }
    stop = (totalSize + ENTRY_EST_SIZE < CACHE_SIZE_LIMIT);
  }

  cacheDesc["total_size"] = totalSize;

  //updateCacheDescFile();

  return firstEvicted;
}

int HipGpuBinaryCache::evictEntry(std::pair<uint32_t, uint32_t>& entry, size_t& totalSize)
{
  // Open the data file and find the required entry.
  std::string dataFilePath = cacheDir + "/" + std::to_string(std::get<0>(entry)) + ".json";
  std::ifstream dataFile(dataFilePath);
  if (!dataFile.is_open()) return false;

  nlohmann::json data;
  std::string cuiFileName, hipfbFileName;
  dataFile >> data;
  dataFile.close();

  nlohmann::json jentry = data[std::get<1>(entry)];

  cuiFileName = jentry["cached_cui"];
  hipfbFileName = jentry["cached_hipfb"];
  std::string subDir = jentry["cache_subdir"];
  int subDirNum = std::stoi(subDir);

  // Remove cached files from the sub-folder.
  std::string subFolderPath = cacheDir + "/" + subDir + "/";
  std::remove((subFolderPath + cuiFileName).c_str());
  std::remove((subFolderPath + hipfbFileName).c_str());

  // Remove the entry from the data file.
  data.erase(std::get<1>(entry));

  // Update the "entry" and "totalSize" argument.
  nlohmann::json next = jentry["next_entry"];
  entry = {next[0], next[1]};
  totalSize -= (size_t) jentry["subdir_size"];

  // Rewrite the data file.
  std::ofstream dataFileW(dataFilePath, std::ios::trunc);
  if (!dataFileW.is_open()) return -1;
  dataFileW << std::setw(4) << data << std::endl;
  dataFileW.close();

  // Invalidate the loaded data file since it could be affected by the eviction.
  //entryData.clear();

  return subDirNum;
}

void HipGpuBinaryCache::addUseHipfbOption(std::string& cmd)
{
    std::string hipfbPath = cacheDir + "/" + cacheEntryInfo.subDir + "/" + cacheEntryInfo.cachedHipfbName;
    cmd += (" --cuda-host-only --use-hipfb=" + hipfbPath);
    if (verbose) {
      std::cout << "[HIPFB CACHE] <<< Using cached hipfb for " << srcFileInfo.name <<
                   " from cache subfolder: " << cacheEntryInfo.subDir << std::endl;
    }
}

bool HipGpuBinaryCache::updateCacheDescFile(bool releaseLock)
{
  // Rewrite the cache description size.
  std::ofstream cacheDescFile(cacheDir + "/" + CACHE_DESC_FILE_NAME, std::ios::trunc);
  if (!cacheDescFile.is_open()) return false;

  //std::cout << "Updating cache desc file for " << srcFileInfo.name << std::endl;

  cacheDescFile << std::setw(4) << cacheDesc;
  cacheDescFile.close();

  //std::cout << std::setw(4) << cacheDesc << std::endl;

  return true;
}

bool HipGpuBinaryCache::initCacheDescFile() 
{
  std::ofstream cacheDescFile(cacheDir + "/" + CACHE_DESC_FILE_NAME, std::ios::trunc);
  if (!cacheDescFile.is_open()) return false;

  cacheDesc["empty_subdirs"] = nlohmann::json::array();
  cacheDesc["pending_subdirs"] = nlohmann::json::array();
  cacheDesc["top_subdir"] = -1;
  cacheDesc["total_size"] = 0;
  cacheDesc["oldest_entry"] = nlohmann::json::array();
  cacheDesc["newest_entry"] = nlohmann::json::array();

  cacheDescFile << std::setw(4) << cacheDesc;
  cacheDescFile.close();

  return true;
}

bool HipGpuBinaryCache::initCache()
{
  struct stat info;
  if (stat(cacheDir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
    // TODO: Move creating a directory to the hipBin_util.h.
    int ret = mkdir(cacheDir.c_str(), S_IRWXU);
    if (ret != 0) return false;
  }

  return initCacheDescFile();
}

bool HipGpuBinaryCache::cleanUpCache()
{
  if (cacheDir.empty()) return false;

  std::ifstream cacheDescFileR(cacheDir + "/" + CACHE_DESC_FILE_NAME);
  if (!cacheDescFileR.is_open()) return false;
  cacheDescFileR >> cacheDesc;
  cacheDescFileR.close();

  // Delete the content of the cache directory.
  for (const auto& entry : fs::directory_iterator(cacheDir)) {
    fs::remove_all(entry.path());
  }

  return initCacheDescFile();
}

#endif  // SRC_HIPBIN_AMD_H_
