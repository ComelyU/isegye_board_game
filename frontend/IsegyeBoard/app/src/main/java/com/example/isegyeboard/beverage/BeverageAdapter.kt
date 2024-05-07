package com.example.isegyeboard.beverage

import android.content.Context
import android.graphics.Color
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.isegyeboard.R
import com.example.isegyeboard.beverage.cart.CartAdapter
import com.example.isegyeboard.beverage.cart.CartClass
import com.example.isegyeboard.beverage.cart.CartManage
import com.example.isegyeboard.beverage.cart.CartUpdateListener
import com.example.isegyeboard.beverage.model.BeverageClass

class BeverageAdapter(private val context: Context, private var dataList: List<BeverageClass>, private val cartAdapter: CartAdapter) :
    RecyclerView.Adapter<BeverageAdapter.BeverageViewHolder>() {

    // ViewHolder 클래스 정의
    inner class BeverageViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView), View.OnClickListener {
        val nameTextView: TextView = itemView.findViewById(R.id.menuName)
        val priceTextView: TextView = itemView.findViewById(R.id.menuPrice)
        val imageView: ImageView = itemView.findViewById(R.id.menuImage)

        init {
            itemView.setOnClickListener(this)
        }

        override fun onClick(v: View?) {
            val menu = dataList[adapterPosition]
            if (menu.isAvailable != 0) {
                val cartItem = CartClass(menu.id, menu.menuName, menu.menuPrice)
                addToCart(cartItem)
                val activity = v?.context as? AppCompatActivity
                activity?.recreate()

                (context as? CartUpdateListener)?.onCartUpdated()
            } else {
                Log.e("List", "e")
            }
        }
    }

    // onCreateViewHolder: ViewHolder 객체를 생성하고 뷰를 연결
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): BeverageViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.layout_menu_item, parent, false)
        return BeverageViewHolder(view)
    }

    // onBindViewHolder: 데이터를 뷰에 연결
    override fun onBindViewHolder(holder: BeverageViewHolder, position: Int) {
        val menuItem = dataList[position]
//            println(gameItem.thumbnailUrl)
        if (menuItem.isAvailable == 0) {
            holder.itemView.setBackgroundColor(Color.LTGRAY)
            holder.nameTextView.setTextColor(Color.GRAY)
            holder.priceTextView.setTextColor(Color.GRAY)
        } else {
            holder.itemView.setBackgroundColor(Color.WHITE)
        }
        holder.nameTextView.text = menuItem.menuName
        holder.priceTextView.text = "${menuItem.menuPrice} 원"
        val imageUrl = "${menuItem.menuImgUrl}"
        Glide.with(context)
            .load(imageUrl)
            .placeholder(R.drawable.ipad)
            .error(R.drawable.chess_black)
            .into(holder.imageView)
    }

    // getItemCount: 데이터 목록의 크기 반환
    override fun getItemCount() = dataList.size

    fun updateData(newData: List<BeverageClass>) {
        dataList = newData
        notifyDataSetChanged()
    }

    private fun addToCart(menu: CartClass) {
        val cart = CartManage.getInstance() // 장바구니 인스턴스 가져오기
        cart.addItem(menu) // 장바구니에 음료 추가
        cartAdapter.updateCartItems(cart.getItems())
        notifyDataSetChanged()
    }
}


