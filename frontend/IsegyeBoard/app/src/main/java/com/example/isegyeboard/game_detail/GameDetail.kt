package com.example.isegyeboard.game_detail

import android.media.AudioManager
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.SeekBar
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

        view.findViewById<ConstraintLayout>(R.id.photoButton).setOnClickListener {
            // 테스트용 임시 네비
            it.findNavController().navigate(R.id.action_main_page_frg_to_gamedetail)
            // 테스트후 삭제
        }

        return view
    }


}