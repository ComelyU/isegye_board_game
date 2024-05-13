package com.example.isegyeboard.room_login

import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.appcompat.app.AlertDialog
import com.example.isegyeboard.baseapi.BaseApi
import com.example.isegyeboard.baseapi.FailureDialog
import com.example.isegyeboard.databinding.FragmentLogoutBinding
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response


class LogoutFragment : Fragment() {

    private lateinit var binding: FragmentLogoutBinding
    private lateinit var sharedPreferences: SharedPreferences

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        binding = FragmentLogoutBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)

        binding.logoutBackButton.setOnClickListener{
            requireActivity().supportFragmentManager.popBackStack()
        }

        binding.logoutButton.setOnClickListener {
            val storeId = binding.logoutStoreNumField.text.toString()
            val roomNum = binding.logoutRoomNumField.text.toString()

            logoutStoreNumCheck(storeId, roomNum)
        }
    }

    private fun logoutStoreNumCheck(storeId: String, roomNum: String) {
        val client = BaseApi.getInstance().create(LoginApi::class.java)

        client.sendStoreInfo(storeId, roomNum).enqueue(object : Callback<Int> {
            override fun onResponse(call : Call<Int>, response: Response<Int>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null) {
//                        Log.d("Login", "login success${response}")
                        Log.d("Login", "login success body ${responseBody}")
                        val roomId = responseBody.toString()
                        logoutFunction(storeId, roomId)
                    } else {
                        Log.d("Login", "login failed $responseBody")
                        FailureDialog.showFailure(requireContext(), "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Login", "request failed $response")
                    FailureDialog.showFailure(requireContext(), "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<Int>, t: Throwable) {
                Log.e("Login", "$t")
                FailureDialog.showFailure(requireContext(), "요청에 실패했습니다.")
            }
        })


    }

    private fun logoutFunction (storeId: String, roomId: String) {
        sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)

        val savedStoreId = sharedPreferences.getString("storeId", "")
        val savedRoomId = sharedPreferences.getString("roomId", "")

        Log.d("logout", "$storeId, $savedStoreId, $roomId, $savedRoomId")
        if (storeId == savedStoreId && roomId == savedRoomId) {
            val intent = Intent(requireContext(), InitialActivity::class.java)
            sharedPreferences.edit().remove("storeId").apply()
            sharedPreferences.edit().remove("roomId").apply()
            startActivity(intent)
            requireActivity().finish()
        } else {
            FailureDialog.showFailure(requireContext(), "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
        }
    }
}