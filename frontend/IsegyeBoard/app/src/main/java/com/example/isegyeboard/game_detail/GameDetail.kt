package com.example.isegyeboard.game_detail

import android.media.AudioManager
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.SeekBar
import android.widget.TextView
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.fragment.app.Fragment
import androidx.navigation.findNavController
import com.example.isegyeboard.R


class GameDetail : Fragment() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        val view = inflater.inflate(R.layout.fragment_gamedetail, container, false)

        view.findViewById<TextView>(R.id.photoButton).setOnClickListener {
            // 테스트용 임시 네비
            it.findNavController().navigate(R.id.action_gamedetail_to_photo)
            // 테스트후 삭제
        }

        return view
    }


}