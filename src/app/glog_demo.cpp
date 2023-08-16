#include <glog/logging.h>

int main(int argc, char* argv[]) {
  // Initialize Google’s logging library.
  FLAGS_stderrthreshold = google::INFO;   // INFO 即 屏幕输出
  FLAGS_colorlogtostderr = true;          // severity 输出显示颜色区分
  google::InitGoogleLogging(argv[0]);     // log file name

  // 0 1 2 3
  LOG(INFO) << "Found "
            << " cookies";
  LOG(WARNING) << "Found "
               << " cookies";
  LOG(ERROR) << "Found "
             << " cookies";
  LOG(FATAL) << "Found "
             << " cookies"; // fatal -> crash

  google::ShutdownGoogleLogging();
}