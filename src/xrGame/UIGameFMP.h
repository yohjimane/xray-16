#pragma once

#include "UIGameMP.h"

class game_cl_freemp;
class CUIStaticItem;

class CUIGameFMP : public UIGameMP
{
private:
	game_cl_freemp *	m_game;
	typedef UIGameMP inherited;

protected:
    CUIStatic* m_stats;

public:
				CUIGameFMP();
	virtual		~CUIGameFMP();

	virtual	void Init(int stage);

	virtual void SetClGame(game_cl_GameState* g);

	virtual void HideShownDialogs();

    virtual void OnFrame() override;

	virtual bool IR_UIOnKeyboardPress(int dik);
};
