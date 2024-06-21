package com.example.controller;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;


import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;
import androidx.navigation.fragment.NavHostFragment;

import com.example.controller.databinding.FragmentFirstBinding;
import com.example.controller.ros.Topic;
import com.example.controller.ros.message.Remote;
import com.example.controller.ros.rosbridge.ROSBridgeClient;


public class FirstFragment extends Fragment {

    private FragmentFirstBinding binding;

    private viewModel viewModel;
    ROSBridgeClient client;

    Topic<Remote> remoteTopic;
    // 弹窗
    private Toast mToast;

    @Override
    public View onCreateView(
            @NonNull LayoutInflater inflater, ViewGroup container,
            Bundle savedInstanceState
    ) {

        binding = FragmentFirstBinding.inflate(inflater, container, false);

        viewModel = new ViewModelProvider(requireActivity()).get(viewModel.class);
        viewModel.getClient().observe(getViewLifecycleOwner(), c -> client = c);
        viewModel.getRemoteTopic().observe(getViewLifecycleOwner(), r -> remoteTopic = r);

        return binding.getRoot();
    }


    public void onViewCreated(@NonNull View view, Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

//        binding.buttonFirst.setOnClickListener(v ->
//                NavHostFragment.findNavController(FirstFragment.this)
//                        .navigate(R.id.action_FirstFragment_to_SecondFragment)
//        );


        binding.directions.btnUp.setOnClickListener(v -> {
            if (client.isConnected()) remoteTopic.publish(new Remote("12"));
            else Toast.makeText(getActivity(), "设备未连接", Toast.LENGTH_SHORT).show();
        });
        binding.directions.btnLeft.setOnClickListener(v -> {
            if (client.isConnected()) remoteTopic.publish(new Remote("14"));
            else Toast.makeText(getActivity(), "设备未连接", Toast.LENGTH_SHORT).show();
        });
        binding.directions.btnRight.setOnClickListener(v -> {
            if (client.isConnected()) remoteTopic.publish(new Remote("16"));
            else Toast.makeText(getActivity(), "设备未连接", Toast.LENGTH_SHORT).show();
        });
        binding.directions.btnDown.setOnClickListener(v -> {
            if (client.isConnected()) remoteTopic.publish(new Remote("18"));
            else Toast.makeText(getActivity(), "设备未连接", Toast.LENGTH_SHORT).show();
        });
        binding.directions.btnCenter.setOnClickListener(v -> {
            if (client.isConnected()) remoteTopic.publish(new Remote("15"));
            else Toast.makeText(getActivity(), "设备未连接", Toast.LENGTH_SHORT).show();
        });
        binding.follow.setOnClickListener(v -> {
            if (client.isConnected()) {
                remoteTopic.publish(new Remote("2"));
                Toast.makeText(getActivity(), "已进入跟随模式", Toast.LENGTH_SHORT).show();
            }
            else Toast.makeText(getActivity(), "设备未连接", Toast.LENGTH_SHORT).show();
        });
        binding.helmet.setOnClickListener(v -> {
            if (client.isConnected()) {
                remoteTopic.publish(new Remote("0"));
                Toast.makeText(getActivity(), "已进入头盔模式", Toast.LENGTH_SHORT).show();
            }
            else Toast.makeText(getActivity(), "设备未连接", Toast.LENGTH_SHORT).show();
        });
    }


    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }

    /**
     * Toast弹窗显示
     * @param text  显示文本
     */
    public void showToast(String text){
        // 若Toast控件未初始化
        if( mToast == null){
            // 则初始化
            mToast = Toast.makeText(getActivity(), text, Toast.LENGTH_SHORT);
        }
        // 否则
        else{
            // 修改显示文本
            mToast.setText(text);
        }
        // 显示
        mToast.show();
    }

}