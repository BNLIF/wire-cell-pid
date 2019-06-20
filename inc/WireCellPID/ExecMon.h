#ifndef WIRECELLPID_EXECMON
#define WIRECELLPID_EXECMON

#include "WireCellPID/TimeKeeper.h"
#include "WireCellPID/MemUsage.h"

namespace WireCellPID {

    /** A helper class combining a TimeKeeper and a MemUsage
     *
     * Use like
     *
     *   ExecMon em("starting");
     *   ...
     *   cerr << em("starting long calculation....") << endl;
     *   do_long_calculation();
     *   cerr << em("...done") << endl;;
     *   ...
     *   cerr << em.summary() << endl;
     */
    class ExecMon {
    public:

	ExecMon(const std::string& msg = "start",
		TimeKeeper::ptime starting_time = boost::posix_time::microsec_clock::local_time());
	~ExecMon();

	/// Record an event.
	std::string operator()(
	    std::string msg = "<tick>",
	    TimeKeeper::ptime now = boost::posix_time::microsec_clock::local_time(),
	    MemUsage::memusage mu = MemUsage::memusage(-1,-1));

	/// Return summary up to now.
	std::string summary() const;

	TimeKeeper tk;
	MemUsage mu;
    };
}
#endif
