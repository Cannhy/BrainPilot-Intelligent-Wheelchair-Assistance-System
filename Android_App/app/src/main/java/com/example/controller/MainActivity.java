package com.example.controller;

import android.os.Bundle;

import com.example.controller.ros.Topic;
import com.example.controller.ros.message.Remote;
import com.example.controller.ros.rosbridge.ROSBridgeClient;
import com.google.android.material.snackbar.Snackbar;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import android.view.LayoutInflater;
import android.view.View;

import androidx.core.app.ActivityCompat;
import androidx.lifecycle.ViewModelProvider;
import androidx.navigation.NavController;
import androidx.navigation.Navigation;
import androidx.navigation.ui.AppBarConfiguration;
import androidx.navigation.ui.NavigationUI;

import com.example.controller.databinding.ActivityMainBinding;

import android.view.Menu;
import android.view.MenuItem;
import android.widget.EditText;
import android.widget.Toast;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity {

    private AppBarConfiguration appBarConfiguration;
    private ActivityMainBinding binding;
    private viewModel viewModel;
    private ROSBridgeClient client;

    private Topic<Remote> remoteTopic;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        client = new ROSBridgeClient();
        remoteTopic = new Topic<Remote>("/remote", Remote.class, client);

        setSupportActionBar(binding.toolbar);

        NavController navController = Navigation.findNavController(this, R.id.nav_host_fragment_content_main);

        appBarConfiguration = new AppBarConfiguration.Builder(navController.getGraph()).build();
        NavigationUI.setupActionBarWithNavController(this, navController, appBarConfiguration);

        binding.fab.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Snackbar.make(view, "欢迎使用电动轮椅远程遥控器", Snackbar.LENGTH_LONG)
                        .setAnchorView(R.id.fab)
                        .setAction("Action", null).show();
            }
        });

        viewModel = new ViewModelProvider(this).get(viewModel.class);
        viewModel.setClient(client);
        viewModel.setRemoteTopic(remoteTopic);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_connect) {
            if (!client.isConnected()) {
                client.connect();
                if (client.isConnected()) {
                    remoteTopic.advertise();
                    Toast.makeText(MainActivity.this,
                            "设备已连接", Toast.LENGTH_SHORT).show();
                } else {
                    Toast.makeText(MainActivity.this,
                            "设备连接失败", Toast.LENGTH_SHORT).show();
                }
            }
            return true;
        }

        if (id == R.id.action_disconnect) {
            if (client.isConnected()) {
                remoteTopic.publish(new Remote("0"));
                remoteTopic.unadvertise();
                client.disconnect();
            }
            Toast.makeText(MainActivity.this,
                    "设备已断开", Toast.LENGTH_SHORT).show();
            return true;
        }

        if (id == R.id.action_ip_edit) {
            // 创建AlertDialog.Builder实例
            AlertDialog.Builder builder = new AlertDialog.Builder(this);

            // 从xml布局文件中加载输入框布局
            LayoutInflater inflater = this.getLayoutInflater();
            View dialogView = inflater.inflate(R.layout.dialog_ip_edit, null);
            builder.setView(dialogView);

            // 获取输入框引用
            EditText editIPText = dialogView.findViewById(R.id.editIPText);
            EditText editPortText = dialogView.findViewById(R.id.editPortText);

            // 设置对话框按钮
            builder.setTitle("手动IP")
                    .setPositiveButton("确认", (dialog, i) -> {
                        // 获取输入框内容
                        String ip = editIPText.getText().toString();
                        String port = editPortText.getText().toString();
                        if(ROSBridgeClient.isValidIP(ip) && ROSBridgeClient.isValidPort(port)) {
                            if (client.isConnected()) client.disconnect();
                            client.manualROSBridgeClient(ip,port);
                            Toast.makeText(MainActivity.this, "已手动设置地址，请重新连接", Toast.LENGTH_SHORT).show();
                        } else {
                            Toast.makeText(MainActivity.this, "地址无效或为空", Toast.LENGTH_SHORT).show();
                        }
                    })
                    .setNegativeButton("取消", (dialog, i) -> dialog.cancel());

            // 显示对话框
            AlertDialog dialog = builder.create();
            dialog.show();
        }

        if (id == R.id.action_ip_reset) {
            if (client.isConnected()) client.disconnect();
            client.resetROSBridgeClient();
            Toast.makeText(MainActivity.this, "已重置为默认IP", Toast.LENGTH_SHORT).show();
        }

        if (id == R.id.action_search) {
            if (client.isConnected()) {
                Toast.makeText(MainActivity.this, "已连接，设备"+client.getUriString(), Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(MainActivity.this, "未连接，设备"+client.getUriString(), Toast.LENGTH_SHORT).show();
            }

        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public boolean onSupportNavigateUp() {
        NavController navController = Navigation.findNavController(this, R.id.nav_host_fragment_content_main);
        return NavigationUI.navigateUp(navController, appBarConfiguration)
                || super.onSupportNavigateUp();
    }



}