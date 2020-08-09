#pragma once
#include <string>
#include <map>
#include <any>
#include <thread>
#include <queue>
#include <mutex>
#include <unordered_set>
#include <boost/function.hpp>

namespace petri_net
{

typedef std::shared_ptr<std::any> Token;

class ThreadSafe
{
public:

  virtual ~ThreadSafe() = default;

  void SafeDo(boost::function<void (ThreadSafe&)> _thing);

private:
  std::mutex mutex_;
};

template<typename T>
class MessageQueue : public std::queue<T>, public ThreadSafe
{};


class Place;
struct StartMsg
{
  const Place& place_;
  const Token token_;
};

class ThreadPool : ThreadSafe
{
public:

  void Kill();

  void SpinOnce();

private:
  std::map<std::thread::id, std::thread> pool_;
  // msg q to give the message of dead threads that need to be removed from the thread pool
  std::queue<std::thread::id> threads_to_remove_;
  // msg q to give the threads that need to be run and with what params
  std::queue<StartMsg> threads_to_start_;
  // bool to signal a kill
  bool kill_ = false;
};



class Transition;
class Place;

class PetriNet
{

public:
  // functions that are used to build the petri net, cannot be called after the net is started
  void AddPlace(const std::string&                         _name,
                const std::unordered_set<std::size_t>& _in_types);

  void SetPlaceCallback(const std::string&             _name,
                        boost::function<const Token
                                       (const Token)>& _callback);

  void AddTransition(const std::string& _t);

  // add an edge from a place to a transition
  void ConnectPlaceTransition(const std::string& _p,
                              const std::string& _t,
                              std::type_info&    _type);

  // add an edge from a transition to a place
  void ConnectTransitionPlace(const std::string&                        _t,
                              const std::string&                        _p,
                              boost::function<const Token
                                (const std::map<std::string&, Token>)>& _token_mux);

  // destroy all semaphores, message quese, child threads, delete all shared dynamic memory
  void Kill(const std::string& _err_msg = std::string("")) const;

  // Places call to signal a close
  void SignalExit(std::thread::id me) const;

  void RequestStart(const std::vector<StartMsg>& _start_msgs) const;

  const Transition& GetTransition(const std::string& _transtion_name) const;

  static const int message_queue_size = 32;

private:
  friend class Transition;

  std::map<std::string, Place>      places_;
  std::map<std::string, Transition> transitions_;

  ThreadPool thread_pool_;
  std::mutex transition_mutex_;
};


class Transition
{
public:
  void TryFire() const;
  void AddInEdge(const std::string& _place);

private:
  friend class PetriNet;
  Transition(PetriNet& _net);

  const PetriNet& net_;
  std::map<std::string, MessageQueue<Token>&> place_type_;

};

class Place
{
public:
  struct OutChannel
  {
    OutChannel(const std::string& _name, MessageQueue<Token>& _msg_q):
    name_(_name), msg_q_(_msg_q)
    {}
    MessageQueue<Token>&            msg_q_;
    std::unordered_set<std::string> transitions_;
    const std::string               name_;
  };

private:

  Place(const std::string&                     _name,
        const std::unordered_set<std::size_t>& _in_types,
        const PetriNet&                        _net);

  void AddOutEdge(std::type_info&    _type,
                  const std::string& _transition);

  void AddOutEdge(std::type_info&      _type,
                  const std::string&   _transition,
                  MessageQueue<Token>& _msg_q);

  void _Callback(const Token _in);

  friend class PetriNet;

  const std::string                        name_;
  const std::unordered_set<std::size_t>    in_types_;
  std::map<std::size_t, OutChannel>        out_edges_;
  const PetriNet&                          net_;

  boost::function<const Token (const Token)> callback_ =
    [](const Token _token)->const Token {return _token;};
};

}
