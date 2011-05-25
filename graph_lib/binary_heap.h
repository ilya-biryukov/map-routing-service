#pragma once

#include <algorithm>
#include <vector>
#include <boost/unordered_map.hpp>

typedef size_t heap_id;

namespace details
{
template<class K, class V>
struct binary_heap_el_t
{
  binary_heap_el_t(K key, V val, heap_id index)
      : key(key), value(val), index(index)
  {
  }
  K key;
  V value;
  heap_id index;
};

template<class K, class V>
bool operator <(binary_heap_el_t<K, V> const & lhs,
                binary_heap_el_t<K, V> const & rhs)
{
  return lhs.key < rhs.key;
}
} // namespace details

template <class K, class V, class Compare = std::less<K> >
struct binary_heap_t
{
  typedef V value_type;
  typedef K key_type;
  typedef size_t size_type;

  binary_heap_t(Compare const & comp = Compare());
  binary_heap_t(size_type capacity, Compare const & comp = Compare());

  heap_id push(std::pair<K, V> const & el);
  heap_id push(K key, V val);
  std::pair<K, V> top() const;
  void extract_top();
  void decrease_key(heap_id id, K new_key);
  bool empty() const
  {
    return heap_data_.empty();
  }
  size_type size() const
  {
    return heap_data_.size();
  }
private:
  typedef std::vector<details::binary_heap_el_t<K, V> > data_storage_t;
  typedef boost::unordered_map<heap_id, typename data_storage_t::size_type>
    index_storage_t;
  data_storage_t heap_data_;
  index_storage_t heap_index_;
  size_t last_id_;
  Compare comp_;

  void swap_elements(typename data_storage_t::size_type e1, 
                     typename data_storage_t::size_type e2);
  void bubble_up(typename data_storage_t::size_type it);
  void sift_down(typename data_storage_t::size_type it);
  typename data_storage_t::size_type node_parent(typename data_storage_t::size_type it);
  typename data_storage_t::size_type node_lchild(typename data_storage_t::size_type it);
  typename data_storage_t::size_type node_rchild(typename data_storage_t::size_type it);
};


// Implementation
template<class K, class V, class Compare>
std::pair<K, V> binary_heap_t<K, V, Compare>::top() const 
{
  details::binary_heap_el_t<K, V> const & e = heap_data_.front();  
  return std::make_pair(e.key, e.value);
}

template<class K, class V, class Compare>
binary_heap_t<K, V, Compare>::binary_heap_t(Compare const & comp)
    : last_id_(-1), comp_(comp)
{
}

template<class K, class V, class Compare>
binary_heap_t<K, V, Compare>::binary_heap_t(size_type capacity,
    Compare const & comp)
    : last_id_(-1), comp_(comp)
{
  heap_data_.reserve(capacity);
  heap_index_.reserve(capacity);
}

template<class K, class V, class Compare>
heap_id binary_heap_t<K, V, Compare>::push(std::pair<K, V> const & el)
{
  return push(el.first, el.second);
}

template<class K, class V, class Compare>
heap_id binary_heap_t<K, V, Compare>::push(K key, V val)
{
  // Insert to a tree leaf
  heap_data_.push_back(details::binary_heap_el_t<K, V>(key, val, ++last_id_));
  // Update index
  heap_index_.insert(std::make_pair(last_id_, heap_data_.size() - 1));
  // Restore heap property
  bubble_up(heap_data_.size() - 1);

  return last_id_;
}

template<class K, class V, class Compare>
void binary_heap_t<K, V, Compare>::extract_top()
{
  assert(!heap_data_.empty());
  // Swap top with the last leaf
  swap_elements(0, heap_data_.size() - 1);
  // Delete the last element and an index of it
  heap_index_.erase(heap_data_.back().index);
  heap_data_.pop_back();
  // Restore heap property
  sift_down(0);
}

template<class K, class V, class Compare>
void binary_heap_t<K, V, Compare>::swap_elements(
  typename data_storage_t::size_type e1,
  typename data_storage_t::size_type e2)
{
  assert(e1 < heap_data_.size());
  assert(e2 < heap_data_.size());
  std::swap(heap_data_[e1], heap_data_[e2]);
  // Update index
  heap_index_[heap_data_[e1].index] = e1;
  heap_index_[heap_data_[e2].index] = e2;
}

template<class K, class V, class Compare>
void binary_heap_t<K, V, Compare>::bubble_up(
  typename data_storage_t::size_type it)
{
  while (it != 0)
  {
    typename data_storage_t::size_type parent = node_parent(it);

    if (comp_(heap_data_[it].key, heap_data_[parent].key))
    {
      swap_elements(it, parent);
      it = parent;
    }
    else
    {
      break;
    }
  }
}

template<class K, class V, class Compare>
typename binary_heap_t<K, V, Compare>::data_storage_t::size_type binary_heap_t <
K, V, Compare >::node_parent(typename data_storage_t::size_type it)
{
  return (it - 1) / 2;
}

template<class K, class V, class Compare>
typename binary_heap_t<K, V, Compare>::data_storage_t::size_type binary_heap_t <
K, V, Compare >::node_lchild(typename data_storage_t::size_type it)
{
  return 2 * it + 1;
}

template<class K, class V, class Compare>
typename binary_heap_t<K, V, Compare>::data_storage_t::size_type binary_heap_t <
K, V, Compare >::node_rchild(typename data_storage_t::size_type it)
{
  return 2 * it + 2;
}

template<class K, class V, class Compare>
void binary_heap_t<K, V, Compare>::sift_down(
  typename data_storage_t::size_type it)
{
  typename data_storage_t::size_type last_nonleaf =
    node_parent(heap_data_.size() - 1);

  while (it <= last_nonleaf)
  {
    typename data_storage_t::size_type lchild = node_lchild(it);
    typename data_storage_t::size_type rchild = node_rchild(it);
    typename data_storage_t::size_type min = it;
    if (lchild < heap_data_.size())
    {
      if (comp_(heap_data_[lchild].key, heap_data_[min].key))
      {
        min = lchild;
      }
    }
    if (rchild < heap_data_.size())
    {
      if (comp_(heap_data_[rchild].key, heap_data_[min].key))
      {
        min = rchild;
      }
    }

    if (min != it)
    {
      swap_elements(min, it);
      it = min;
    }
    else
    {
      break;
    }
  }
}

template<class K, class V, class Compare>
void binary_heap_t<K, V, Compare>::decrease_key(heap_id id, K new_key)
{
  typename index_storage_t::iterator it = heap_index_.find(id);
  if (it != heap_index_.end())
  {
    heap_data_[it->second].key = new_key;
    bubble_up(it->second);
  }
}
