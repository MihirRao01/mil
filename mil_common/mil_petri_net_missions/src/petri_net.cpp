#include <mil_petri_net_missions/petri_net.hpp>

namespace petri_net
{

void PetriNet::AddPlace(const std::string&                        _name,
                        const std::unordered_set<std::size_t>& _in_types)
{
  places_.emplace(_name, Place(_name, _in_types, *this));
}

void PetriNet::SetPlaceCallback(const std::string&                          _name,
                                boost::function<const Token (const Token)>& _callback)
{
  places_.at(_name).callback_ = _callback;
}

void PetriNet::AddTransition(const std::string& _name)
{
  transitions_.emplace(_name, Transition(*this));
}

void PetriNet::ConnectPlaceTransition(const std::string&     _p,
                                      const std::string&     _t,
                                      std::type_info&        _type)
{
  places_.at(_p).AddOutEdge(_type, _t);
}

void PetriNet::ConnectTransitionPlace(const std::string&                        _t,
                                      const std::string&                        _p,
                                      boost::function<const Token
                                        (const std::map<std::string&, Token>)>& _token_mux)
{

}

void PetriNet::Kill(const std::string& _err_msg) const
{
  // signal to the thread pool to kill everything
  thread_pool_.Kill();
}

const Transition& PetriNet::GetTransition(const std::string& _transtion_name) const
{
  return transitions_.at(_transtion_name);
}


Place::Place(const std::string&                     _name,
      const std::unordered_set<std::size_t>& _in_types,
      const PetriNet&                        _net):
      name_(_name), in_types_(_in_types), net_(_net)
{}

void Place::_Callback(const Token _token)
{
  // see if the type of the given token is an accepted type
  // if it is not
  if (in_types_.find(_token->type().hash_code()) == in_types_.end())
  {
    // if not, then error and send a signal to the Petri Net to kill who whole thing
    net_.Kill();//name_ + ": Given token of type " + _token->type().name() + " is not supported\n");
    return;
  }

  const auto token = callback_(_token);
  // see if the returned Token type is accepted by any of our out edges
  // if not, kill the net
  if (out_edges_.find(token->type().hash_code()) == out_edges_.end())
  {
    // error the type returned is wrong
    net_.Kill();//name_ + ": Returned Token of type " + token->type().name() +
//               " is not supported\n");
    return;
  }
  auto& out = out_edges_.at(token->type().hash_code());
  // enqueue token in the proper message queue
  out.msg_q_.SafeDo(
  [&](ThreadSafe& _ts)->void
  {
    //MessageQueue<Token>& msg_q_ = std::dynamic_cast<MesageQueue<Token>&>(self);
    auto& msg_q = dynamic_cast<MessageQueue<Token>&>(_ts);
    msg_q.push(token);
  });
  // iterate through and call the transitions that belong to that type
  for (auto& transition : out.transitions_)
  {
    net_.GetTransition(transition).TryFire();
  }
}


void Place::AddOutEdge(std::type_info&    _type,
                       const std::string& _transition)
{
  if (out_edges_.find(_type.hash_code()) == out_edges_.end())
  {
    throw std::runtime_error("new type requires new MessageQueue");
  }
  if (out_edges_.at(_type.hash_code()).transitions_.insert(_transition).second == false)
  {
    throw std::runtime_error("This edge already exists");
  }
}

void Place::AddOutEdge(std::type_info&      _type,
                       const std::string&   _transition,
                       MessageQueue<Token>& _msg_q)
{
  if (out_edges_.find(_type.hash_code()) != out_edges_.end())
  {
    throw std::runtime_error("this type already has a MessageQueue");
  }
  out_edges_.emplace(_type.hash_code(), OutChannel(name_ + "->" + _type.name(), _msg_q));
}

Transition::Transition(PetriNet& _net) : net_(_net)
{}

void Transition::TryFire() const
{
  // wait on the transition semaphore
  net_.transition_mutex_.lock();
  // check all the input message quese for any data



  net_.trnasition

}

void Transition::AddInEdge(const std::string& _place)
{}

void ThreadSafe::SafeDo(boost::function<void (ThreadSafe&)> _thing)
{
  mutex_.lock();
  _thing(*this);
  mutex_.unlock();
}


void PetriNet::ThreadPool::Kill()
{
SafeDo([](ThreadSafe& _tf)->void
{
  auto& self = dynamic_cast<ThreadPool&>(_tf);
  self.kill_ = true;
});
}


void PetriNet::ThreadPool::SpinOnce()
{
SafeDo([](ThreadSafe& _tf)->void
{
  auto& self = dynamic_cast<ThreadPool&>(_tf);
  // check the kill
  if (self.kill_ == true)
  {
    // terminate all threads
    for(auto& i : self.pool_)
    {
      self.pool_.erase(i.first);
    }
    return;
  }
  // join the threads that need to be joined
  while(self.threads_to_remove_.size() > 0)
  {
    self.pool_.at(self.threads_to_remove_.front()).join();
    self.pool_.erase(self.threads_to_remove_.front());
    self.threads_to_remove_.pop();
  }
  // start the threads that need to be started
  while(self.threads_to_start_.size() > 0)
  {
    auto& msg = self.threads_to_start_.front();
    std::thread new_thread(msg.place_.callback_, msg.token_);
    self.pool_.insert(std::make_pair(new_thread.get_id(), std::move(new_thread)));
    self.threads_to_start_.pop();
  }
});
}

}
