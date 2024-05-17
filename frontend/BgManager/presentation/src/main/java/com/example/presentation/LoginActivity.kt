package com.example.presentation

import android.content.Context
import android.content.Intent
import android.os.Bundle
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import com.example.presentation.R
import com.example.presentation.databinding.ActivityLoginBinding
import com.example.presentation.databinding.ActivityMainBinding

class LoginActivity : AppCompatActivity() {

    private var _binding: ActivityLoginBinding? = null

    private val binding
        get() = _binding!!
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        _binding = ActivityLoginBinding.inflate(layoutInflater)
        setContentView(binding.root)

        val storeId = binding.storeNumField.text.toString().trim()

        binding.loginButton.setOnClickListener{
            login(storeId)
        }
    }

    private fun login(storeId:String) {
        val sharedPref = getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)

        val editor = sharedPref.edit()
        editor.putString("storeId", storeId)
        editor.apply()

        val intent = Intent(this, MainActivity::class.java)
        startActivity(intent)
    }
}