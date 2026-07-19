#include <cv_bridge/cv_bridge.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sonar_image_proc/sonar_image_msg_interface.h>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include "sonar_image_proc/SonarDrawer.h"

namespace po = boost::program_options;

using std::string;
using std::vector;

class OutputWrapper {
 public:
  virtual void write(
      const std::shared_ptr<const marine_acoustic_msgs::msg::ProjectedSonarImage> &msg,
      const cv::Mat &mat) = 0;
  virtual ~OutputWrapper() = default;
};

class BagOutput : public OutputWrapper {
 public:
  BagOutput(const std::string &bagfile, const std::string &topic)
      : topic_(topic) {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bagfile;
    storage_options.storage_id = "sqlite3";
    
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options, converter_options);
    
    // Create topic metadata
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = topic;
    topic_metadata.type = "sensor_msgs/msg/Image";
    topic_metadata.serialization_format = "cdr";
    writer_->create_topic(topic_metadata);
  }

  void write(const std::shared_ptr<const marine_acoustic_msgs::msg::ProjectedSonarImage> &msg,
             const cv::Mat &mat) override {
    cv_bridge::CvImage img_bridge(msg->header, "rgb8", mat);
    auto output_msg = img_bridge.toImageMsg();

    // Serialize and write the message
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    serialization.serialize_message(output_msg.get(), serialized_msg.get());
    
    // Use the original message timestamp
    rclcpp::Time timestamp(msg->header.stamp);
    
    writer_->write(serialized_msg, topic_, "sensor_msgs/msg/Image", timestamp);
  }

  ~BagOutput() {
    if (writer_) {
      writer_->close();
    }
  }

 private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string topic_;
};

void print_help(const po::options_description &description) {
  std::cout << "Usage:" << std::endl;
  std::cout << std::endl;
  std::cout << "   bag2sonar [options] <input file(s)>" << std::endl;
  std::cout << std::endl;
  std::cout << description;
  exit(0);
}

int main(int argc, char **argv) {
  po::options_description public_description("Draw sonar from a bagfile");

  // The following code with three differente po::option_descriptions in a
  // slight-of-hand to hide the positional argment "input-files"
  // otherwise it shows up in print_help() which is ugly
  //
  // clang-format off
  public_description.add_options()
    ("help,h", "Display this help message")
    ("logscale,l", po::bool_switch()->default_value(true), "Do logscale")
    ("min-db", po::value<float>()->default_value(0), "Min db")
    ("max-db", po::value<float>()->default_value(0), "Max db")
    ("osd", po::bool_switch()->default_value(true), "If set, include the on-screen display in output")
    ("output-bag,o", po::value<string>(), "Name of output bagfile")
    ("output-topic,t", po::value<string>()->default_value("/drawn_sonar"), "Topic for images in output bagfile");

  po::options_description hidden_description("");
  hidden_description.add_options()
    ("input-files", po::value<std::vector<std::string>>()->required(), "Input files");
  // clang-format on

  po::options_description full_description("");
  full_description.add(public_description).add(hidden_description);

  po::positional_options_description p;
  p.add("input-files", -1);

  po::variables_map vm;

  try {
    po::store(po::command_line_parser(argc, argv)
                  .options(full_description)
                  .positional(p)
                  .run(),
              vm);
    po::notify(vm);

  } catch (const boost::program_options::required_option &e) {
    // This exception occurs when a required option (input-files) isn't supplied
    // Catch that and display a help message instead.
    print_help(public_description);
  } catch (const std::exception &e) {
    // Catch any other random errors and exit
    std::cerr << e.what() << std::endl;
    exit(-1);
  }

  if (vm.count("help")) {
    print_help(public_description);
  } else if (vm.count("input-files")) {
    if (vm.count("input-files") > 1) {
      std::cerr << "Can only process one file at a time" << std::endl;
      exit(-1);
    }

    std::vector<std::shared_ptr<OutputWrapper>> outputs;

    if (vm.count("output-bag")) {
      outputs.push_back(std::make_shared<BagOutput>(
          vm["output-bag"].as<string>(), vm["output-topic"].as<string>()));
    }

    sonar_image_proc::SonarDrawer sonar_drawer;
    std::unique_ptr<sonar_image_proc::SonarColorMap> color_map(
        new sonar_image_proc::InfernoColorMap);

    std::vector<std::string> files =
        vm["input-files"].as<std::vector<std::string>>();
    for (const std::string &file : files) {
      std::cout << "Processing input file " << file << std::endl;

      rosbag2_cpp::Reader reader;
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = file;
      storage_options.storage_id = "sqlite3";
      
      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";
      
      reader.open(storage_options, converter_options);

      int count = 0;

      // Set up deserialization
      rclcpp::Serialization<marine_acoustic_msgs::msg::ProjectedSonarImage> serialization;

      while (reader.has_next()) {
        auto bag_message = reader.read_next();
        
        // Check if this is a ProjectedSonarImage message
        if (bag_message->topic_name.find("sonar") == std::string::npos &&
            bag_message->topic_name.find("image") == std::string::npos) {
          continue;  // Skip non-sonar topics
        }

        try {
          rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
          auto msg = std::make_shared<marine_acoustic_msgs::msg::ProjectedSonarImage>();
          serialization.deserialize_message(&serialized_msg, msg.get());

          sonar_image_proc::SonarImageMsgInterface interface(msg);

          // Validate the data buffer covers ranges*bearings*elem before drawing;
          // a truncated/malformed ping is otherwise read out of bounds.
          {
            size_t elem = 0;
            if (msg->image.dtype == msg->image.DTYPE_UINT8) elem = 1;
            else if (msg->image.dtype == msg->image.DTYPE_UINT16) elem = 2;
            else if (msg->image.dtype == msg->image.DTYPE_UINT32) elem = 4;
            const size_t need = static_cast<size_t>(interface.nRanges()) *
                                static_cast<size_t>(interface.nBearings()) * elem;
            if (elem == 0 || msg->image.data.size() < need) {
              std::cerr << "Skipping malformed sonar image ("
                        << msg->image.data.size() << " bytes < " << need
                        << " required)" << std::endl;
              continue;
            }
          }

          if (vm["logscale"].as<bool>()) {
            interface.do_log_scale(vm["min-db"].as<float>(),
                                   vm["max-db"].as<float>());
          }

          cv::Mat rect_mat =
              sonar_drawer.drawRectSonarImage(interface, *color_map);

          cv::Mat sonar_mat = sonar_drawer.remapRectSonarImage(interface, rect_mat);

          cv::Mat out_mat;
          if (vm["osd"].as<bool>()) {
            out_mat = sonar_drawer.drawOverlay(interface, sonar_mat);
          } else {
            out_mat = sonar_mat;
          }

          for (auto &output : outputs) {
            output->write(msg, out_mat);
          }

          count++;

          if ((count % 100) == 0) {
            std::cout << "Processed " << count << " sonar frames" << std::endl;
          }
        } catch (const std::exception &e) {
          // Skip messages that can't be deserialized as ProjectedSonarImage
          continue;
        }
      }

      reader.close();
      std::cout << "Processed " << count << " total sonar frames from " << file << std::endl;
    }
  }

  exit(0);
}
