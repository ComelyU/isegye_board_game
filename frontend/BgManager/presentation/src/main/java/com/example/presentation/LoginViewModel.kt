package com.example.presentation

import android.content.Intent
import androidx.lifecycle.viewModelScope
import com.example.presentation.base.BaseViewModel
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import javax.inject.Inject

class LoginViewModel @Inject constructor(
//    private val loginUseCase: LoginUseCase,
) : BaseViewModel<Unit>() {

//    fun login() {
//        viewModelScope.launch {
//            val response = loginUseCase.invoke()
//            if (response.isSuccess) {
//                val data = response.getOrThrow()
//
//                val intent = Intent(this, LoginActivity::class.java)
//                startActivity(intent)
//            }
//        }
//    }
}