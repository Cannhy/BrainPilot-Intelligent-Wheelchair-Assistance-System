package com.example.controller;

// SharedViewModel.java
import androidx.lifecycle.LiveData;
import androidx.lifecycle.MutableLiveData;
import androidx.lifecycle.ViewModel;

import com.example.controller.ros.Topic;
import com.example.controller.ros.message.Remote;
import com.example.controller.ros.rosbridge.ROSBridgeClient;

public class viewModel extends ViewModel {
    private final MutableLiveData<ROSBridgeClient> client = new MutableLiveData<>();
    private final MutableLiveData<Topic<Remote>> remoteTopic = new MutableLiveData<>();

    public void setClient(ROSBridgeClient client) {
        this.client.setValue(client);
    }

    public LiveData<ROSBridgeClient> getClient() {
        return client;
    }

    public void setRemoteTopic(Topic<Remote> remoteTopic) {
        this.remoteTopic.setValue(remoteTopic);
    }

    public MutableLiveData<Topic<Remote>> getRemoteTopic() {
        return remoteTopic;
    }

}
