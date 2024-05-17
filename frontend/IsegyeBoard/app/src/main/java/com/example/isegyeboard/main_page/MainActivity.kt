package com.example.isegyeboard.main_page

import android.content.Context
import android.os.Bundle
import android.util.Log
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.navigation.findNavController
import com.example.isegyeboard.MainNavDirections
import com.example.isegyeboard.R
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.ShowDialog
import com.example.isegyeboard.databinding.ActivityMainBinding
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        supportActionBar?.hide()

        binding = ActivityMainBinding.inflate(layoutInflater)
        val view = binding.root
        setContentView(view)

        binding.logoButton.setOnClickListener {
            val action = MainNavDirections.actionGlobalHome()
            findNavController(R.id.mainFragmentView).navigate(action)
        }

        val sharedPreferences = getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val storeId = sharedPreferences.getString("storeId", "1")

        // 화면이 생성될 때 getStoreInfo 함수 호출
        getStoreInfo(storeId.toString())
    }


    private fun getStoreInfo(storeId: String) {
        val client = BaseApi.getInstance().create(StoreInfoApi::class.java)
        client.getStoreInfo(storeId).enqueue(object : Callback<StoreClass> {
            override fun onResponse(call : Call<StoreClass>, response: Response<StoreClass>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null) {
                        Log.d("StoreInfo", "get recommend item success $response, $responseBody")
                        binding.rightBottomTextView.text = responseBody.storeName

                    } else {
                        Log.d("StoreInfo", "Recommend empty $response, ${response.body()}")
                        ShowDialog.showFailure(this@MainActivity, "지점정보가 비어있습니다.")
                    }
                } else {
                    Log.d("StoreInfo", "Recommend failed $response")
                    ShowDialog.showFailure(this@MainActivity, "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<StoreClass>, t: Throwable) {
                Log.e("StoreInfo", "$t")
                ShowDialog.showFailure(this@MainActivity, "요청에 실패했습니다.")
            }
        })
    }

}