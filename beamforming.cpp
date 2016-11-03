#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <boost/program_options.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <csignal>
#include <ctime>
#include <fstream>
#include <cmath>
#include <sys/socket.h>
#include <string>

#define pi 3.141592654
#define c 299792458.0

namespace po = boost::program_options;
using boost::asio::ip::udp;
/***********************************************************************
 * Signal handlers
 **********************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}
size_t len = 0;
void handler(
  const boost::system::error_code& error, // Result of operation.
  size_t length          // Number of bytes received.
  ) {
	  len = length;
  } 

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();
	
    //variables to be set by po
    std::string args, wave_type, ant, subdev, ref, pps, otw, channel_list, file, addr, remote_addr;
    double rate, freq, gain, bw, send_time;
    float phi, x_dist, y_dist, lmd, deg_step, delay, ant_rot_res;
	int x_num, y_num, port;
    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("rate", po::value<double>(&rate), "rate of outgoing samples")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("subdev", po::value<std::string>(&subdev), "subdevice specification")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo, gpsdo)")
        ("pps", po::value<std::string>(&pps), "PPS source (internal, external, mimo, gpsdo)")
        ("otw", po::value<std::string>(&otw)->default_value("sc16"), "specify the over-the-wire sample mode")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channels to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("int-n", "tune USRP with integer-N tuning")
		("x_dist", po::value<float>(&x_dist)->default_value(5.08), "distance between two antennas w.r.t to x-axis (in cm)")
		("y_dist", po::value<float>(&y_dist)->default_value(5.08), "distance between two antennas w.r.t to y-axis (in cm)")
		("x_num", po::value<int>(&x_num)->default_value(8), "number of antennas w.r.t to x-axis")
		("y_num", po::value<int>(&y_num)->default_value(4), "number of antennas w.r.t to y-axis")
		("step-resolution", po::value<float>(&deg_step)->default_value(1), "Resolution of increment")
		("delay", po::value<float>(&delay)->default_value(0), "Beam switch delay")
		("send-time", po::value<double>(&send_time)->default_value(5), "frame rate of the display (fps)")
		("file", po::value<std::string>(&file), "name of the file to take samples from")
		("addr", po::value<std::string>(&addr), "address for UDP connection")
		("server_addr", po::value<std::string>(&remote_addr), "server address for UDP connection")
		("port", po::value<int>(&port), "port number for UDP connection")
		("resolution for antenna rotation", po::value<float>(&ant_rot_res)->default_value(5), "Beam switch delay")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX Waveforms %s") % desc << std::endl;
        return ~0;
    }

	if (!vm.count("file")) {
		std::cout << "Enter file name...!!!\n";
		return ~0;
	}
	if (!vm.count("addr") || !vm.count("port")) {
		std::cout << "Enter port and(or) address for UDP conneciton (required for sync)...!!!\n";
		return ~0;
	}
    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = boost::lexical_cast<int>(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels())
            throw std::runtime_error("Invalid channel(s) specified.");
        else
            channel_nums.push_back(boost::lexical_cast<int>(channel_strings[ch]));
    }


    //Lock mboard clocks
    usrp->set_clock_source(ref);

    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) usrp->set_tx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate()/1e6) << std::endl << std::endl;

    //set the center frequency
    if (not vm.count("freq")){
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    }

    for(size_t ch = 0; ch < channel_nums.size(); ch++) {
        std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_tx_freq(tune_request, channel_nums[ch]);
        std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq(channel_nums[ch])/1e6) << std::endl << std::endl;

        //set the rf gain
        if (vm.count("gain")){
            std::cout << boost::format("Setting TX Gain: %f dB...") % gain << std::endl;
            usrp->set_tx_gain(gain, channel_nums[ch]);
            std::cout << boost::format("Actual TX Gain: %f dB...") % usrp->get_tx_gain(channel_nums[ch]) << std::endl << std::endl;
        }

        //set the analog frontend filter bandwidth
        if (vm.count("bw")){
            std::cout << boost::format("Setting TX Bandwidth: %f MHz...") % bw << std::endl;
            usrp->set_tx_bandwidth(bw, channel_nums[ch]);
            std::cout << boost::format("Actual TX Bandwidth: %f MHz...") % usrp->get_tx_bandwidth(channel_nums[ch]) << std::endl << std::endl;
        }

        //set the antenna
        if (vm.count("ant")) usrp->set_tx_antenna(ant, channel_nums[ch]);
    }

    boost::this_thread::sleep(boost::posix_time::seconds(1)); //allow for some setup time

	//create a transmit streamer
	//linearly map channels (index0 = channel0, index1 = channel1, ...)
	uhd::stream_args_t stream_args("fc32", otw);
	stream_args.channels = channel_nums;
	uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

	std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
	if (channel_nums.size() > 1)
	{
		// Sync times
		if (pps == "mimo")
		{
			UHD_ASSERT_THROW(usrp->get_num_mboards() == 2);

			//make mboard 1 a slave over the MIMO Cable
			usrp->set_time_source("mimo", 1);

			//set time on the master (mboard 0)
			usrp->set_time_now(uhd::time_spec_t(0.0), 0);

			//sleep a bit while the slave locks its time to the master
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
		else
		{
			if (pps == "internal" or pps == "external" or pps == "gpsdo")
				usrp->set_time_source(pps);
			usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
			boost::this_thread::sleep(boost::posix_time::seconds(1)); //wait for pps sync pulse
		}
	}
	else
	{
		usrp->set_time_now(0.0);
	}

	//Check Ref and LO Lock detect
	std::vector<std::string> sensor_names;
	const size_t tx_sensor_chan = channel_list.empty() ? 0 : boost::lexical_cast<size_t>(channel_list[0]);
	sensor_names = usrp->get_tx_sensor_names(tx_sensor_chan);
	if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
		uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked", tx_sensor_chan);
		std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
		UHD_ASSERT_THROW(lo_locked.to_bool());
	}
	const size_t mboard_sensor_idx = 0;
	sensor_names = usrp->get_mboard_sensor_names(mboard_sensor_idx);
	if ((ref == "mimo") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
		uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", mboard_sensor_idx);
		std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string() << std::endl;
		UHD_ASSERT_THROW(mimo_locked.to_bool());
	}
	if ((ref == "external") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
		uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", mboard_sensor_idx);
		std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string() << std::endl;
		UHD_ASSERT_THROW(ref_locked.to_bool());
	}



	
	std::ifstream infile(file.c_str(), std::ifstream::binary);
	infile.seekg(0, infile.end);
	size_t num_tx_samps = infile.tellg()/sizeof(std::complex<float>);
	infile.seekg(0, infile.beg);
	std::vector<std::complex<float> > temp_buff(num_tx_samps);
	infile.read((char*)&temp_buff.front(), temp_buff.size()*sizeof(std::complex<float>));
	//allocate a buffer which we re-use for each channel
	//if (!infile.is_open()) {std::cout << "File not read...\n";}
	std::cout << std::endl << "Tx samps: " << num_tx_samps << std::endl;
	std::vector<std::vector<std::complex<float> > > buff(channel_nums.size(), std::vector<std::complex<float> > (num_tx_samps));
	std::vector<std::complex<float> *> buffs;
	for (int ch = 0; ch < channel_nums.size(); ch++) buffs.push_back(&buff[ch].front());

	std::signal(SIGINT, &sig_int_handler);
	std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

	
	
	std::string flag = "TX";
	boost::asio::io_service io_service;
	udp::endpoint local_endpoint(boost::asio::ip::address::from_string(addr), port);
	udp::endpoint server_endpoint(boost::asio::ip::address::from_string(remote_addr), port);
	udp::socket socket(io_service, local_endpoint);
	if (!socket.is_open()) {socket.open(udp::v4());}
	while (len == 0 and not stop_signal_called) {
		len = socket.send_to(boost::asio::buffer(flag.c_str(), flag.size()), server_endpoint);
	}
	flag.clear();
	std::cout << "Starting reception...\n";
	//std::string rec;
	boost::array<char, uhd::transport::udp_simple::mtu> rec;
	//boost::array<char, 5> temp = ;
	len = 0;
	//rec.resize(uhd::transport::udp_simple::mtu);
	while (len <= 0 and not stop_signal_called) {
		len = socket.receive_from(boost::asio::buffer(rec), server_endpoint);
	}
	std::cout << "Received something...\n";
	std::cout.write(rec.data(),len);
	std::cout << std::endl;
	len = 0;
	
	
	
	
	
	uhd::tx_metadata_t md;
	lmd = static_cast<float>(c/freq);
	std::vector<std::complex<float> > Wn;
	//while (usrp->get_time_now() < 3.0) {}
	//while (usrp->get_time_now() < 10.0 and not stop_signal_called) {}
	while(not stop_signal_called){
		for (phi = 0; phi <= 360; phi += deg_step){
			if (stop_signal_called) break;
			usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
			boost::this_thread::sleep(boost::posix_time::seconds(2)); //wait for pps sync pulse
			
			std::cout << "Angle: " << phi << std::endl;
			
			for (int y = 0; y < y_num; y++) {
				for (int x = 0; x < x_num; x++) {
					if ((x + y)%2 == 0) {
						std::complex<float> expval(0, (2 * pi * 1/lmd)*((std::cos(phi*pi / 180.0)*((x)*(x_dist*0.01))) + (std::sin(phi*pi / 180.0)*(((y_num) - 1 - (y))*(y_dist*0.01)))));
						Wn.push_back(std::exp(expval));
					}
				}
			}
			
			std::cout << "\n\n";
			for (int i = 0; i < channel_nums.size(); i++){
				std::cout << atan2(Wn[i].imag(),Wn[i].real()) * 180.0/pi << "\t";
			}
			std::cout << "\n\n";
			
			if (phi != 360) {
				for (size_t ch = channel_nums[0]; ch < channel_nums.back(); ch++) {
					for (size_t n = 0; n < buff.at(ch).size(); n++){
						buff.at(ch - channel_nums[0])[n] = Wn[ch]*temp_buff[n];
					}
				}
			}
			else {
				for (size_t ch = channel_nums[0]; ch < channel_nums.back(); ch++) {
					for (size_t n = 0; n < buff.at(ch).size(); n++){
						buff.at(ch - channel_nums[0])[n] = temp_buff[n];
					}
				}
			}			
			
			
			for (int i = 0; i < 360/ant_rot_res; i++) {
				if (stop_signal_called) break;
				while (len <= 0 and not stop_signal_called) {
					len = socket.receive_from(boost::asio::buffer(rec), server_endpoint);
				}
				std::cout.write(rec.data(),len);
				std::cout << std::endl;
				len = 0;
				//a->uhd::time_spec_t::time_spec_t(0.0);
				//usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
				//boost::this_thread::sleep(boost::posix_time::seconds(2)); //wait for pps sync pulse
				//send data until the signal handler gets called
				//or if we accumulate the number of samples specified (unless it's 0)
				//uint64_t num_acc_samps = 0;
				//uhd::time_spec_t checking = usrp->get_time_now();
				//while (usrp->get_time_now() < checking + uhd::time_spec_t(2.0)) {}
				//int tx_flag = 0;
				md.start_of_burst = true;
				md.end_of_burst   = false;
				md.has_time_spec = true;
				//uhd::time_spec_t sig_send_time = usrp->get_time_now();
				//uhd::time_spec_t sd_time = uhd::time_spec_t(send_time);
				std::cout << "Sending samples...\n";
				flag += "Start RX";
				while (len == 0 and not stop_signal_called) {
					len = socket.send_to(boost::asio::buffer(flag.c_str(), flag.size()), server_endpoint);
				}
				flag.clear();
				md.time_spec = usrp->get_time_now() + uhd::time_spec_t(0.01);
				socket.non_blocking(true);
				while(socket.receive_from(boost::asio::buffer(rec), server_endpoint) <= 0 and not stop_signal_called){
					//send the entire contents of the buffer
					tx_stream->send(buffs, buff.at(0).size(), md);
					md.start_of_burst = false;
					md.has_time_spec = false;
				}
				md.end_of_burst = true;
				tx_stream->send("", 0, md);
				std::cout.write(rec.data(),len);
				std::cout << std::endl;
				len = 0;
				socket.non_blocking(false);
				std::cout << "Samples sent\n";
			}
			Wn.clear();
			Wn.resize(0);
		}
	}


	//send a mini EOB packet
	//md.end_of_burst = true;
    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}


