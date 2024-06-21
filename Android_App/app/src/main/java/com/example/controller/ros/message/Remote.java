package com.example.controller.ros.message;

@MessageType(string = "remote")
public class Remote extends Message {
    public String data;
    public Remote(String data){
        this.data = data;
    }
}
