/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <android-base/logging.h>

#include <cerrno>
#include <unordered_map>
#include <vector>

namespace qti::audio::core {

/**
 * Collection of items with a unique key (id)
 **/
template <class T>
class AudioCollection {
   public:
    explicit AudioCollection(const AudioCollection& c);
    explicit AudioCollection(AudioCollection&& c);
    AudioCollection() = default;

    const std::vector<T>& getCollection() const;

    // Copy and move assignment for the collection
    AudioCollection& operator=(const AudioCollection& c);
    AudioCollection& operator=(AudioCollection&& c);
    AudioCollection& operator=(const std::vector<T>& ports);
    AudioCollection& operator=(std::vector<T>&& ports);

    // Comparison operators with collection of the same type
    bool operator==(const AudioCollection& c);
    bool operator!=(const AudioCollection& c);

    // Appends entries from the passed collection to this collection
    void operator+=(const AudioCollection& c);
    void operator+=(const std::vector<T>& c);

    // Access the collection
    T& operator[](int32_t id);
    const T& at(int32_t id) const;
    bool contains(int32_t id) const;
    T& front();
    T& back();

    // Copies the passed entry to the end of collection.
    // An entry with the same id is erased.
    // Returns a reference to the inserted entry.
    const T& push_back(const T& entry);

    // Moves the passed entry to the end of collection.
    // An entry with the same id is erased.
    // Returns a reference to the inserted entry.
    const T& emplace_back(T&& entry);

    // Erases element with the given id
    // Returns the id of an entry following the erased, returns -1 if the
    // erased entry is the last one or if the collection is empty
    int32_t erase(int32_t id);

    void clear();

    // Capacity of the collection
    bool empty() const;
    void reserve(size_t size);
    size_t capacity() const;
    size_t size() const;

    using CVIterator = typename std::vector<T>::const_iterator;
    CVIterator cbegin() const { return mCollection.cbegin(); }
    CVIterator cend() const { return mCollection.cend(); }

    using VIterator = typename std::vector<T>::iterator;
    VIterator begin() { return mCollection.begin(); }
    VIterator end() { return mCollection.end(); }

   private:
    std::vector<T> mCollection;
    // key:T's id, value: index in mCollection
    std::unordered_map<int32_t, uint32_t> mIdToEntryMap;
};

template <class T>
AudioCollection<T>::AudioCollection(const AudioCollection& c)
    : mCollection(c.mCollection), mIdToEntryMap(c.mIdToEntryMap) {}

template <class T>
AudioCollection<T>::AudioCollection(AudioCollection&& c)
    : mCollection(std::move(c.mCollection)),
      mIdToEntryMap(std::move(c.mIdToEntryMap)) {}

template <class T>
const std::vector<T>& AudioCollection<T>::getCollection() const {
    return mCollection;
}

template <class T>
bool AudioCollection<T>::contains(int32_t id) const {
    return mIdToEntryMap.find(id) != mIdToEntryMap.end();
}

template <class T>
const T& AudioCollection<T>::at(int32_t id) const {
    CHECK_EQ(contains(id), true)
        << __func__ << ":id " << id << " not present in collection";
    return mCollection.at(mIdToEntryMap.at(id));
}

template <class T>
T& AudioCollection<T>::front() {
    CHECK_EQ(empty(), false)
        << __func__ << ": Invalid when collection is empty";
    return mCollection.front();
}

template <class T>
T& AudioCollection<T>::back() {
    CHECK_EQ(empty(), false)
        << __func__ << ": Invalid when collection is empty";
    return mCollection.back();
}

template <class T>
int32_t AudioCollection<T>::erase(int32_t id) {
    if (empty()) {
        return -1;
    }
    CHECK_EQ(contains(id), true)
        << __func__ << ":id " << id << " not present in collection";
    auto orderIndex = mIdToEntryMap[id];
    auto itNext = mCollection.erase(mCollection.begin() + orderIndex);
    mIdToEntryMap.erase(id);
    std::for_each(itNext, mCollection.end(),
                  [&](const auto& e) { mIdToEntryMap[e.id] = orderIndex++; });
    return itNext != mCollection.end() ? (*itNext).id : -1;
}

template <class T>
void AudioCollection<T>::clear() {
    mCollection.clear();
    mIdToEntryMap.clear();
}

template <class T>
AudioCollection<T>& AudioCollection<T>::operator=(const AudioCollection<T>& c) {
    mCollection = c.mCollection;
    mIdToEntryMap = c.mIdToEntryMap;
    return *this;
}

template <class T>
AudioCollection<T>& AudioCollection<T>::operator=(AudioCollection<T>&& c) {
    mCollection = std::move(c.mCollection);
    mIdToEntryMap = std::move(c.mIdToEntryMap);
    return *this;
}

template <class T>
T& AudioCollection<T>::operator[](int32_t id) {
    CHECK_EQ(contains(id), true)
        << __func__ << ":id " << id << " not present in collection";
    return mCollection.at(mIdToEntryMap.at(id));
}

template <class T>
bool AudioCollection<T>::operator==(const AudioCollection<T>& c) {
    return mCollection == c.mCollection;
}

template <class T>
bool AudioCollection<T>::empty() const {
    return mCollection.empty();
}

template <class T>
size_t AudioCollection<T>::capacity() const {
    return mCollection.capacity();
}

template <class T>
size_t AudioCollection<T>::size() const {
    return mCollection.size();
}

template <class T>
void AudioCollection<T>::reserve(size_t size) {
    if (size <= capacity()) {
        return;
    }
    mCollection.reserve(size);
}

template <class T>
const T& AudioCollection<T>::push_back(const T& entry) {
    if (contains(entry.id)) {
        erase(entry.id);
    }
    mCollection.push_back(entry);
    mIdToEntryMap[entry.id] = size() - 1;
    return at(entry.id);
}

template <class T>
const T& AudioCollection<T>::emplace_back(T&& entry) {
    if (contains(entry.id)) {
        erase(entry.id);
    }
    mCollection.emplace_back(std::move(entry));
    mIdToEntryMap[entry.id] = size() - 1;
    return at(entry.id);
}

template <class T>
AudioCollection<T>& AudioCollection<T>::operator=(const std::vector<T>& ports) {
    clear();
    std::for_each(ports.begin(), ports.end(),
                  [&](const auto& port) { push_back(port); });
    return *this;
}

template <class T>
AudioCollection<T>& AudioCollection<T>::operator=(std::vector<T>&& ports) {
    clear();
    for (auto&& port : ports) {
        emplace_back(std::move(port));
    }
    return *this;
}

template <class T>
void AudioCollection<T>::operator+=(const AudioCollection& c) {
    auto insertEntry = [&](const auto& entry) { push_back(entry); };
    std::for_each(c.mCollection.begin(), c.mCollection.end(), insertEntry);
}

template <class T>
void AudioCollection<T>::operator+=(const std::vector<T>& c) {
    auto insertEntry = [&](const auto& entry) { push_back(entry); };
    std::for_each(c.begin(), c.end(), insertEntry);
}

}  // namespace qti::audio::core
