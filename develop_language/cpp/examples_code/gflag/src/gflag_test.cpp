#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
using namespace std;
// using namespace gflags;
namespace gflags = google;
DEFINE_string(gpus, "", 
	"Optional; run in GPU mode on given device IDs separated by ','."
    "Use '-gpu all' to run on all available GPUs. The effective training "
    "batch size is multiplied by the number of devices.");
DEFINE_int32(iterations, 50, 
	"The number of iterations to run.");

namespace klm{

void GlobalInit(int* pargc, char*** pargv) {
  // Google flags.
  ::gflags::ParseCommandLineFlags(pargc, pargv, true);
  // Google logging.
  ::google::InitGoogleLogging(*(pargv)[0]);
  // Provide a backtrace on segfault.
  ::google::InstallFailureSignalHandler();
}
}

int main(int argc, char** argv)
{
	// Print output to stderr (while still logging).
  	FLAGS_alsologtostderr = 1;

	klm::GlobalInit(&argc, &argv);
	gflags::SetUsageMessage("command line brew\n"
      "usage: caffe <command> <args>\n\n"
      "commands:\n"
      "  train           train or finetune a model\n"
      "  test            score a model\n"
      "  device_query    show GPU diagnostic information\n"
      "  time            benchmark model execution time");

	// test
	gflags::ShowUsageWithFlagsRestrict(argv[0], "tools/caffe");

	if (FLAGS_gpus == "all") {
		cout << "Using GPUs 1,2,3,4,5,6,7,8 "<< endl;
	}else {
		cout << "Using GPUs " << FLAGS_gpus << endl;
	}
	cout << "Iterations " << FLAGS_iterations << endl;
	LOG(INFO) << "Iterations " << FLAGS_iterations;
	LOG(ERROR) << "\t" << FLAGS_iterations;
	//LOG(FATAL) << "Invalid signal effect \""<< FLAGS_iterations << "\" was specified";

	return 0;
}